package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ZephyrSubsystem {

    // Tuning
    private static final double AVERAGE_VELOCITY_IN_PER_SEC = 280.0; // Tune this! Real velocity usually ~150-200 in/s
    private static final double BLUE_BASKET_X = 72.0;
    private static final double BLUE_BASKET_Y = 72.0;
    private static final double RED_BASKET_X = -72.0;
    private static final double RED_BASKET_Y = -72.0;

    // hw
    private static final double TURRET_MAX_DEG = 235.0;
    private static final double TURRET_MIN_DEG = -125.0;
    // Check your gear ratio: 5:1 UltraPlanetary is usually closer to 5.23:1
    private static final double TICKS_PER_REV = 28.0; // Standard bare motor
    private static final double GEAR_RATIO = 5.0; // External reduction
    private static final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    private DcMotorEx turretMotor;
    private DcMotorEx shooterMotor;
    private Servo leftHood, rightHood;
    private SparkFunOTOS otos;
    private AprilTagLimelight limelight;

    private enum TurretState { TRACKING, UNWINDING }
    private TurretState currentState = TurretState.TRACKING;

    // Logic Variables
    private double unwindTargetAngle = 0;
    private double unwindStartHeading = 0;
    private double targetX = BLUE_BASKET_X;
    private double targetY = BLUE_BASKET_Y;
    private int targetTagID = 20;

    // Own PID for Turret
    private PIDController turretPID = new PIDController(0.005, 0.0001, 0.0002);

    private double debugVirtualDist = 0;

    public ZephyrSubsystem(HardwareMap hardwareMap, SparkFunOTOS otosRef, AprilTagLimelight limelightRef) {
        this.otos = otosRef;
        this.limelight = limelightRef;

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

    public void setAlliance(boolean isBlue) {
        if (isBlue) {
            targetX = BLUE_BASKET_X;
            targetY = BLUE_BASKET_Y;
            targetTagID = 20;
        } else {
            targetX = RED_BASKET_X;
            targetY = RED_BASKET_Y;
            targetTagID = 24;
        }
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

    private FiringSolution solveFiringSolution() {
        SparkFunOTOS.Pose2D pos = otos.getPosition();
        SparkFunOTOS.Pose2D vel = otos.getVelocity(); // Robot Centric (Forward/Strafe)
        double headingRad = pos.h; // OTOS uses Radians usually

        double dx = targetX - pos.x;
        double dy = targetY - pos.y;
        double realDist = Math.hypot(dx, dy);
        double timeOfFlight = realDist / AVERAGE_VELOCITY_IN_PER_SEC;

        double vFieldX = (vel.x * Math.cos(headingRad)) - (vel.y * Math.sin(headingRad));
        double vFieldY = (vel.x * Math.sin(headingRad)) + (vel.y * Math.cos(headingRad));

        double virtX = targetX - (vFieldX * timeOfFlight);
        double virtY = targetY - (vFieldY * timeOfFlight);

        double vecX = virtX - pos.x;
        double vecY = virtY - pos.y;

        double targetFieldAngle = Math.toDegrees(Math.atan2(vecY, vecX));
        double virtualDist = Math.hypot(vecX, vecY);
        debugVirtualDist = virtualDist;

        double relativeAngle = targetFieldAngle - Math.toDegrees(headingRad);
        relativeAngle = normalizeAngle(relativeAngle);

        if (limelight.GetLimelightId() == targetTagID) {
            // Get the raw angle from the camera (TX is usually relative to camera center)
            double cameraError = limelight.GetTX();

            double currentTurret = getTurretDegrees();

            // If camera sees target at +5 deg, and turret is at +90, target is at +95
            // BUT, Limelight TX is "Error from crosshair".
            // If TX is +5 (right), we need to move turret +5 more.
            double visionTarget = currentTurret - cameraError;

            // Add the "Lead" offset calculated from the feedforward
            double physicsCorrection = targetFieldAngle - Math.toDegrees(Math.atan2(dy, dx));
            visionTarget += physicsCorrection;

            // Complimentary Filter: Trust Vision (80%) + Physics (20%)
            relativeAngle = (visionTarget * 0.8) + (relativeAngle * 0.2);
            relativeAngle = normalizeAngle(relativeAngle);
        }

        double rpm = calculateRPMFromCurve(virtualDist);
        double hood = 0.45; // You should probably make a curve for this too

        return new FiringSolution(relativeAngle, rpm, hood);
    }

    private void moveTurretToAngle(double targetAngle) {
        double currentAngle = getTurretDegrees();

        // Soft Limits
        if (targetAngle > TURRET_MAX_DEG || targetAngle < TURRET_MIN_DEG) {
            // If we hit the limit, check if we are already there
            if (currentAngle > TURRET_MAX_DEG || currentAngle < TURRET_MIN_DEG) {
                initiateUnwind(targetAngle);
            }
            return;
        }

        double power = turretPID.calculate(targetAngle, currentAngle);
        turretMotor.setPower(power);
    }

    private void initiateUnwind(double blockedTargetAngle) {
        currentState = TurretState.UNWINDING;
        if (blockedTargetAngle > 0) {
            unwindTargetAngle = blockedTargetAngle - 360;
        } else {
            unwindTargetAngle = blockedTargetAngle + 360;
        }
    }

    private void executeUnwind() {
        // Just use PID to get to the new unwound angle
        double power = turretPID.calculate(unwindTargetAngle, getTurretDegrees());
        turretMotor.setPower(power);

        // If we are close enough, switch back to tracking mode
        if (Math.abs(getTurretDegrees() - unwindTargetAngle) < 5.0) {
            currentState = TurretState.TRACKING;
        }
    }

    private double calculateRPMFromCurve(double dist) {
        // Warning: This polynomial might be dangerous if dist is very small or very large
        // Ensure dist is clamped or the curve is safe
        double x = dist - 3.0;
        return (0.001 * Math.pow((x - 53), 4)) + 3300;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void setShooterRPM(double rpm) {
        // 28 ticks per rev is for a BARE motor.
        // 1:1
        double velocity = (rpm / 60.0) * 28.0;
        shooterMotor.setVelocity(velocity);
    }

    private void setHoodPos(double pos) {
        leftHood.setPosition(pos);
        rightHood.setPosition(pos);
    }

    public double getTurretDegrees() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    // Help classes

    private static class FiringSolution {
        public double turretAngle, rpm, hoodAngle;
        public FiringSolution(double angle, double rpm, double hood) {
            this.turretAngle = angle;
            this.rpm = rpm;
            this.hoodAngle = hood;
        }
    }

    // Fixed PID Controller with its own timer
    public static class PIDController {
        double kP, kI, kD;
        double lastError = 0;
        double integral = 0;
        double maxIntegral = 1.0;
        ElapsedTime timer = new ElapsedTime(); // Internal timer

        public PIDController(double p, double i, double d) {
            this.kP = p; this.kI = i; this.kD = d;
            timer.reset();
        }

        public double calculate(double target, double current) {
            double error = target - current;
            double dt = timer.seconds();
            timer.reset(); // Reset immediately after reading

            // Guard against divide by zero or huge time steps
            if (dt < 0.001) dt = 0.001;
            if (dt > 0.1) dt = 0.1;

            integral += error * dt;
            // Clamp integral to prevent "Windup"
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

            double derivative = (error - lastError) / dt;
            lastError = error;

            return (error * kP) + (integral * kI) + (derivative * kD);
        }
    }
}