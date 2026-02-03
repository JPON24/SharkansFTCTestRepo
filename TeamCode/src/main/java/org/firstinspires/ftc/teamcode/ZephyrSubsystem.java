package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Seattle Solvers Fraudulent Code
 */
public class ZephyrSubsystem {

    private static final double AVERAGE_VELOCITY_IN_PER_SEC = 280.0; // tune pls

    private static final double TURRET_MAX_DEG = 235.0;
    private static final double TURRET_MIN_DEG = -125.0;
    private static final double TICKS_PER_REV = 145.1;
    private static final double GEAR_RATIO = 5.0;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor;
    private Servo leftHood, rightHood;
    private SparkFunOTOS otos;
    private AprilTagLimelight limelight;

    private enum TurretState {
        TRACKING,
        UNWINDING,
        IDLE
    }
    private TurretState currentState = TurretState.TRACKING;

    // Unwind State Data
    private double unwindTargetAngle = 0;
    private double unwindStartHeading = 0;

    // Target Data
    private double targetX = 0; // Field Centric X of tower
    private double targetY = 0; // Field Centric Y of tower
    private int targetTagID = 20; // -1 means no tag selected

    // PID Controllers
    private PIDController turretPID = new PIDController(0.002, 0.0001, 0.0);
    private ElapsedTime timer = new ElapsedTime();

    // Telemetry get my balls
    private double debugVirtualDist = 0;
    private double debugVirtualX = 0;
    private double debugVirtualY = 0;

    public ZephyrSubsystem(HardwareMap hardwareMap, SparkFunOTOS otosRef, AprilTagLimelight limelightRef) {
        this.otos = otosRef;
        this.limelight = limelightRef; // Pass initialized limelight if possible, or init here

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooter");
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftHood = hardwareMap.get(Servo.class, "leftHood");
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        leftHood.setDirection(Servo.Direction.REVERSE);
    }

    public void update() {
        if (currentState == TurretState.UNWINDING) {
            executeUnwind();
            return;
        }

        FiringSolution solution = solveFiringSolution();

        setShooterRPM(solution.rpm);
        setHoodPos(solution.hoodAngle);

        moveTurretToAngle(solution.turretAngle);
    }

    /**
     * Paint my balls blue
     */
    private FiringSolution solveFiringSolution() {
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        SparkFunOTOS.Pose2D vel = otos.getVelocity();

        double dx = targetX - pos.x;
        double dy = targetY - pos.y;
        double realDist = Math.hypot(dx, dy);
        double timeOfFlight = realDist / AVERAGE_VELOCITY_IN_PER_SEC;

        double virtX = targetX - (vel.x * timeOfFlight);
        double virtY = targetY - (vel.y * timeOfFlight);


        debugVirtualX = virtX;
        debugVirtualY = virtY;

        double vecX = virtX - pos.x;
        double vecY = virtY - pos.y;

        double targetFieldAngle = Math.toDegrees(Math.atan2(vecY, vecX));

        double virtualDist = Math.hypot(vecX, vecY);
        debugVirtualDist = virtualDist;

        double robotHeading = Math.toDegrees(pos.h);
        double relativeAngle = targetFieldAngle - robotHeading;

        relativeAngle = normalizeAngle(relativeAngle);

        if (limelight.GetLimelightId() == targetTagID) {
            double realAngle = Math.toDegrees(Math.atan2(dy, dx));
            double leadOffset = targetFieldAngle - realAngle;

            double visionAngle = (getTurretDegrees() - limelight.GetTX()) + leadOffset;

            relativeAngle = (visionAngle * 0.8) + (relativeAngle * 0.2);
        }

        double rpm = calculateRPMFromCurve(virtualDist);
        double hood = 0.45;

        return new FiringSolution(relativeAngle, rpm, hood);
    }

    private void moveTurretToAngle(double targetAngle) {
        double currentAngle = getTurretDegrees();
        double currentTicks = turretMotor.getCurrentPosition();
        double targetTicks = targetAngle * TICKS_PER_DEGREE;

        int maxTicks = (int)(TURRET_MAX_DEG * TICKS_PER_DEGREE);
        int minTicks = (int)(TURRET_MIN_DEG * TICKS_PER_DEGREE);

        if (targetTicks > maxTicks || targetTicks < minTicks) {
            if (currentTicks > maxTicks || currentTicks < minTicks) {
                initiateUnwind(targetAngle);
            } else {
                turretMotor.setPower(0);
            }
            return;
        }

        // Run PID
        double power = turretPID.calculate(targetAngle, currentAngle);
        turretMotor.setPower(power);
    }

    private void initiateUnwind(double blockedTargetAngle) {
        currentState = TurretState.UNWINDING;
        // Find the "other way around" (flip 360)
        double currentHeading = Math.toDegrees(otos.getPosition().h);

        if (blockedTargetAngle > 0) {
            unwindTargetAngle = blockedTargetAngle - 360;
        } else {
            unwindTargetAngle = blockedTargetAngle + 360;
        }
        unwindStartHeading = currentHeading;
    }

    private void executeUnwind() {
        double currentHeading = Math.toDegrees(otos.getPosition().h);
        // Adjust target based on how much the robot turned while unwinding
        double headingDelta = currentHeading - unwindStartHeading;
        double dynamicTarget = unwindTargetAngle - headingDelta;

        // Simple P-Control to get there fast
        double power = turretPID.calculate(dynamicTarget, getTurretDegrees());
        turretMotor.setPower(power);

        if (Math.abs(getTurretDegrees() - dynamicTarget) < 5.0) {
            currentState = TurretState.TRACKING;
        }
    }


    private double calculateRPMFromCurve(double dist) {
        // Your regression equation: f(x) = 0.001(x-53)^4 + 3300
        // Adjusted for Limelight offset constant
        double x = dist - 3.0;
        return (0.001 * Math.pow((x - 53), 4)) + 3300;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }


    public void setTarget(double x, double y, int aprilTagID) {
        this.targetX = x;
        this.targetY = y;
        this.targetTagID = aprilTagID;
    }

    private void setShooterRPM(double rpm) {
        double velocity = (rpm / 60.0) * 28.0; // 28 counts/rev for motor tune this
        shooterMotor.setVelocity(velocity);
    }

    private void setHoodPos(double pos) {
        leftHood.setPosition(pos);
        rightHood.setPosition(pos);
    }

    public double getTurretDegrees() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public double getVirtualDist() { return debugVirtualDist; }
    public String getTurretState() { return currentState.toString(); }


    private static class FiringSolution {
        public double turretAngle;
        public double rpm;
        public double hoodAngle;

        public FiringSolution(double angle, double rpm, double hood) {
            this.turretAngle = angle;
            this.rpm = rpm;
            this.hoodAngle = hood;
        }
    }

    private class PIDController {
        double kP, kI, kD;
        double lastError = 0;
        double integral = 0;
        double maxIntegral = 1.0;

        public PIDController(double p, double i, double d) {
            this.kP = p; this.kI = i; this.kD = d;
        }

        public double calculate(double target, double current) {
            double error = target - current;
            double dt = timer.seconds();
            timer.reset();

            integral += error * dt;
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

            double derivative = (error - lastError) / dt;
            lastError = error;

            return (error * kP) + (integral * kI) + (derivative * kD);
        }
    }
}