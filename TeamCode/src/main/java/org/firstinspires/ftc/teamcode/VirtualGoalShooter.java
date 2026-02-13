package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.global.Ballistics;
import org.firstinspires.ftc.teamcode.global.constants;
import org.firstinspires.ftc.teamcode.global.Vector2D;
import org.firstinspires.ftc.teamcode.global.InterpLUT;
import org.firstinspires.ftc.teamcode.global.LinearMath;
import org.firstinspires.ftc.teamcode.global.PIDController;


public class VirtualGoalShooter {

    Ballistics Ballistics;

    public static final Vector2D BLUE_BASKET = new Vector2D(constants.ZEPHYR_BLUE_BASKET_X, constants.ZEPHYR_BLUE_BASKET_Y);
    public static final Vector2D RED_BASKET = new Vector2D(constants.ZEPHYR_RED_BASKET_X, constants.ZEPHYR_RED_BASKET_Y);

    private final double WHEEL_RADIUS_INCHES = 1.378;
    private final double GEAR_RATIO = 1.0;
    private final double HOOD_MIN_ANGLE = 20.0;
    private final double HOOD_MAX_ANGLE = 60.0;

    private final double TURRET_MAX_DEG = constants.TURRET_MAX_DEG;
    private final double TURRET_MIN_DEG = constants.TURRET_MIN_DEG;
    private final double TICKS_PER_DEGREE = constants.TURRET_TICKS_PER_DEGREE;
    private final double TICKS_PER_REV_SHOOTER = constants.SHOOTER_COUNTS_PER_MOTOR_REV;

    private double baseP = constants.TURRET_KP;
    private double baseI = constants.TURRET_KI;
    private double baseD = constants.TURRET_KD;
    private double baseF = constants.TURRET_KF;

    private double normalP = 100, boostP = 200, rpmDropThreshold = constants.SHOOTER_RPM_DROP_THRESHOLD;

    private DcMotorEx turretMotor, rightShooter;
    private Servo leftHood, rightHood;
    private SparkFunOTOS otos;
    private AprilTagLimelight limelight;

    private Vector2D targetPos = BLUE_BASKET;
    private InterpLUT rpmTable = new InterpLUT("RPM");
    private InterpLUT hoodTable = new InterpLUT("Hood");

    private PIDController turretPID;

    public enum TurretState { TRACKING, UNWINDING }
    private TurretState currentState = TurretState.TRACKING;
    private double unwindTargetAngle = 0;

    // Flywheel state - NEW
    private boolean flywheelEnabled = false;
    private double targetRPM = 0;

    // Store last firing solution - NEW
    private FiringSolution lastSolution = null;

    public void init(HardwareMap hardwareMap, SparkFunOTOS otosRef, AprilTagLimelight limelightRef) {
        this.otos = otosRef;
        this.limelight = limelightRef;

        turretPID = new PIDController(baseP, baseI, baseD, baseF);
        turretPID.setMaxIntegral(1.0);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setDirection(DcMotorEx.Direction.FORWARD);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftHood = hardwareMap.get(Servo.class, "leftHood");
        leftHood.setDirection(Servo.Direction.REVERSE);
        rightHood = hardwareMap.get(Servo.class, "rightHood");

        //jacob got that
        rpmTable.add(65, 3300);
        rpmTable.add(73.5, 3350);
        rpmTable.add(77, 3400);
        rpmTable.add(81, 3450.0);
        rpmTable.add(85, 3700.0);
        rpmTable.add(90, 3800);

        hoodTable.add(56, 0.45);
        hoodTable.add(71, 0.5);
        hoodTable.add(77, 0.45);
        hoodTable.add(81, 0.55);
        hoodTable.add(85, 0.1);
    }

    public void setAlliance(boolean isBlue) {
        this.targetPos = isBlue ? BLUE_BASKET : RED_BASKET;
    }

    /**
     * Call this in your loop - it automatically tracks the turret and hood
     * but does NOT spin the flywheel unless spinUpShooter() has been called
     */
    public void update() {
        // Always update shooter PIDF regardless of flywheel state
        updateShooterPIDF();

        // Handle unwinding state
        if (currentState == TurretState.UNWINDING) {
            executeUnwind();
            // Still calculate solution for hood/flywheel even during unwind
            lastSolution = solveFiringSolution();
            if (lastSolution.validShot) {
                setHoodPos(lastSolution.hoodAngle);
                if (flywheelEnabled) {
                    setShooterRPM(lastSolution.rpm);
                }
            }
            return;
        }

        FiringSolution solution = solveFiringSolution();
        lastSolution = solution;

        if (solution.validShot) {
            setHoodPos(solution.hoodAngle);
            moveTurretToAngle(solution.turretAngle);

            if (flywheelEnabled) {
                setShooterRPM(solution.rpm);
            } else {
                setShooterRPM(0);
            }
        } else {
            turretMotor.setPower(0);
            setShooterRPM(0);
        }
    }

    /**
     * Call this to start spinning the flywheel
     * Turret will continue to track automatically
     */
    public void spinUpShooter() {
        flywheelEnabled = true;
    }

    /**
     * Call this to stop the flywheel
     * Turret will continue to track automatically
     */
    public void stopShooter() {
        flywheelEnabled = false;
        setShooterRPM(0);
        targetRPM = 0;
    }

    /**
     * Check if the shooter is ready to fire
     * @param rpmTolerance How close to target RPM (default: 100)
     * @return true if flywheel is at speed and shot is valid
     */
    public boolean isReadyToShoot(double rpmTolerance) {
        if (!flywheelEnabled || lastSolution == null || !lastSolution.validShot) {
            return false;
        }

        double currentRPM = getCurrentRPM();
        double error = Math.abs(targetRPM - currentRPM);

        return error < rpmTolerance && targetRPM > 0;
    }

    /**
     * Overload with default tolerance
     */
    public boolean isReadyToShoot() {
        return isReadyToShoot(100.0);
    }

    /**
     * Check if turret is aimed at target
     * @param angleTolerance How close to target angle in degrees (default: 2.0)
     * @return true if turret is within tolerance
     */
    public boolean isTurretOnTarget(double angleTolerance) {
        if (lastSolution == null || !lastSolution.validShot) {
            return false;
        }

        double currentAngle = getTurretDegrees();
        double error = Math.abs(lastSolution.turretAngle - currentAngle);

        return error < angleTolerance;
    }

    /**
     * Overload with default tolerance
     */
    public boolean isTurretOnTarget() {
        return isTurretOnTarget(2.0);
    }

    /**
     * Check if everything is ready for a shot
     */
    public boolean isFullyReady() {
        return isReadyToShoot() && isTurretOnTarget();
    }

    private FiringSolution solveFiringSolution() {
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        SparkFunOTOS.Pose2D vel = otos.getVelocity();
        double chassisHeading = pos.h;

        double dx = targetPos.x - pos.x;
        double dy = targetPos.y - pos.y;
        double rawDist = Math.hypot(dx, dy);

        double tableRPM = rpmTable.get(rawDist);
        double tableHood = hoodTable.get(rawDist);

        double launchVelocity = Ballistics.toLinearVelocity(tableRPM, WHEEL_RADIUS_INCHES, GEAR_RATIO);
        double launchAngle = hoodToDegrees(tableHood);

        double timeOfFlight = Ballistics.calculateTimeOfFlight(rawDist, launchVelocity, launchAngle);


        double vFieldX = (vel.x * Math.cos(chassisHeading)) - (vel.y * Math.sin(chassisHeading));
        double vFieldY = (vel.x * Math.sin(chassisHeading)) + (vel.y * Math.cos(chassisHeading));

        double virtX = targetPos.x - (vFieldX * timeOfFlight);
        double virtY = targetPos.y - (vFieldY * timeOfFlight);

        double virtDx = virtX - pos.x;
        double virtDy = virtY - pos.y;

        double targetFieldAngle = Math.toDegrees(Math.atan2(virtDy, virtDx));
        double relativeTurretAngle = targetFieldAngle - Math.toDegrees(chassisHeading);

        if (limelight.GetLimelightId() != 0) {
            double currentTurret = getTurretDegrees();
            double visionTarget = currentTurret - limelight.GetTX();
            relativeTurretAngle = LinearMath.lerp(relativeTurretAngle, visionTarget, 0.2);
        }

        relativeTurretAngle = LinearMath.angleWrap(relativeTurretAngle);

        double virtualDist = Math.hypot(virtDx, virtDy);

        return new FiringSolution(
                relativeTurretAngle,
                rpmTable.get(virtualDist),
                hoodTable.get(virtualDist),
                virtualDist > 10 && virtualDist < 140
        );
    }

    private double hoodToDegrees(double servoPos) {
        return LinearMath.lerp(HOOD_MIN_ANGLE, HOOD_MAX_ANGLE, servoPos);
    }

    private void moveTurretToAngle(double targetAngle) {
        double currentAngle = getTurretDegrees();

        if (targetAngle > TURRET_MAX_DEG || targetAngle < TURRET_MIN_DEG) {
            double altAngle = (targetAngle > 0) ? targetAngle - 360 : targetAngle + 360;
            if (currentAngle > TURRET_MAX_DEG - 10 || currentAngle < TURRET_MIN_DEG + 10) {
                initiateUnwind(altAngle);
                return;
            }
            targetAngle = LinearMath.clamp(targetAngle, TURRET_MIN_DEG, TURRET_MAX_DEG);
        }

        double error = targetAngle - currentAngle;

        if (Math.abs(error) > 20) {
            turretPID.setPIDF(baseP * 3.0, baseI, baseD, baseF);
        } else if (Math.abs(error) > 10) {
            turretPID.setPIDF(baseP * 2.0, baseI, baseD, baseF);
        } else {
            turretPID.setPIDF(baseP, baseI, baseD, baseF);
        }

        double power = turretPID.update(targetAngle, currentAngle);
        turretMotor.setPower(power);
    }

    private void initiateUnwind(double target) {
        currentState = TurretState.UNWINDING;
        unwindTargetAngle = target;
    }

    private void executeUnwind() {
        turretPID.setPIDF(baseP * 1.5, 0, 0.3, 0);

        double power = turretPID.update(unwindTargetAngle, getTurretDegrees());
        turretMotor.setPower(power);

        if (Math.abs(unwindTargetAngle - getTurretDegrees()) < 5.0) {
            currentState = TurretState.TRACKING;
        }
    }

    private void updateShooterPIDF() {
        double error = targetRPM - getCurrentRPM();
        if (error > rpmDropThreshold && targetRPM > 0) {
            rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(boostP, 0, 1, 1));
        } else {
            rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(normalP, 0, 1, 1));
        }
    }

    public double getTurretDegrees() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    private void setShooterRPM(double rpm) {
        this.targetRPM = rpm;
        double vel = (rpm / 60.0) * TICKS_PER_REV_SHOOTER;
        rightShooter.setVelocity(vel);
    }

    private void setHoodPos(double pos) {
        leftHood.setPosition(pos);
        rightHood.setPosition(pos);
    }

    public double getCurrentRPM() {
        return (rightShooter.getVelocity() / TICKS_PER_REV_SHOOTER) * 60.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Get the current turret target angle
     */
    public double getTargetTurretAngle() {
        return lastSolution != null ? lastSolution.turretAngle : 0;
    }

    /**
     * Get the turret error (how far off target)
     */
    public double getTurretError() {
        if (lastSolution == null) return 0;
        return lastSolution.turretAngle - getTurretDegrees();
    }

    /**
     * Check if flywheel is currently enabled
     */
    public boolean isFlywheelEnabled() {
        return flywheelEnabled;
    }

    private static class FiringSolution {
        double turretAngle, rpm, hoodAngle;
        boolean validShot;
        public FiringSolution(double t, double r, double h, boolean v) {
            this.turretAngle = t; this.rpm = r; this.hoodAngle = h; this.validShot = v;
        }
    }
}