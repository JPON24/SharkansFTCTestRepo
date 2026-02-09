package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.sharklib.core.util.math.LinearMath;
import com.sharklib.core.util.math.InterpLUT;
import com.sharklib.core.util.math.geometry.Vector2D;
import com.sharklib.core.control.PIDController;

import org.firstinspires.ftc.teamcode.global.constants;

public class VirtualGoalShooter {

    public static final Vector2D BLUE_BASKET = new Vector2D(constants.ZEPHYR_BLUE_BASKET_X, constants.ZEPHYR_BLUE_BASKET_Y);
    public static final Vector2D RED_BASKET = new Vector2D(constants.ZEPHYR_RED_BASKET_X, constants.ZEPHYR_RED_BASKET_Y);
    public static double PROJECTILE_SPEED = constants.ZEPHYR_AVG_VELOCITY;

    private final double TURRET_MAX_DEG = constants.TURRET_MAX_DEG;
    private final double TURRET_MIN_DEG = constants.TURRET_MIN_DEG;
    private final double TICKS_PER_DEGREE = constants.TURRET_TICKS_PER_DEGREE;
    private final double TICKS_PER_REV_SHOOTER = constants.SHOOTER_COUNTS_PER_MOTOR_REV;

    private double baseP = 0.006;
    private double baseI = 0.0;
    private double baseD = 0.0002;
    private double baseF = 0.0;

    private double normalP = 100, boostP = 200, rpmDropThreshold = 300;

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
    private double targetRPM = 0;

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

        rpmTable.add(0.0, 3100.0);
        rpmTable.add(38.5, 3300.0);
        rpmTable.add(48.5, 3300.0);
        rpmTable.add(68.5, 3450.0);
        rpmTable.add(83.5, 3700.0);
        rpmTable.add(129.0, 4600.0);

        hoodTable.add(0.0, 0.65);
        hoodTable.add(37.0, 0.45);
        hoodTable.add(62.0, 0.45);
        hoodTable.add(72.0, 0.15);
        hoodTable.add(129.0, 0.0);
    }

    public void setAlliance(boolean isBlue) {
        this.targetPos = isBlue ? BLUE_BASKET : RED_BASKET;
    }

    public void update() {
        updateShooterPIDF();

        if (currentState == TurretState.UNWINDING) {
            executeUnwind();
            return;
        }

        FiringSolution solution = solveFiringSolution();

        if (solution.validShot) {
            setShooterRPM(solution.rpm);
            setHoodPos(solution.hoodAngle);
            moveTurretToAngle(solution.turretAngle);
        } else {
            turretMotor.setPower(0);
            setShooterRPM(0);
        }
    }

    private FiringSolution solveFiringSolution() {
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        SparkFunOTOS.Pose2D vel = otos.getVelocity();
        double chassisHeading = pos.h;

        double vFieldX = (vel.x * Math.cos(chassisHeading)) - (vel.y * Math.sin(chassisHeading));
        double vFieldY = (vel.x * Math.sin(chassisHeading)) + (vel.y * Math.cos(chassisHeading));

        double dx = targetPos.x - pos.x;
        double dy = targetPos.y - pos.y;
        double rawDist = Math.hypot(dx, dy);
        double timeOfFlight = rawDist / PROJECTILE_SPEED;

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
        turretPID.setPIDF(baseP * 1.5, 0, 0, 0);

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

    public double getTargetRPM() { return targetRPM; }

    private static class FiringSolution {
        double turretAngle, rpm, hoodAngle;
        boolean validShot;
        public FiringSolution(double t, double r, double h, boolean v) {
            this.turretAngle = t; this.rpm = r; this.hoodAngle = h; this.validShot = v;
        }
    }
}