package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.global.util.math.Ballistics;
import org.firstinspires.ftc.teamcode.global.constants;
import org.firstinspires.ftc.teamcode.global.util.math.Vector2D;
import org.firstinspires.ftc.teamcode.global.util.math.InterpLUT;
import org.firstinspires.ftc.teamcode.global.util.math.LinearMath;
import org.firstinspires.ftc.teamcode.global.control.PIDController;
import org.firstinspires.ftc.teamcode.global.control.KalmanFilter;


public class VirtualGoalShooter {

    Ballistics Ballistics;

    public enum StartPosition{
        CLOSE_BLUE,
        CLOSE_RED,
        FAR_BLUE,
        FAR_RED,
        NO_POSITION
    }

    public static final Vector2D BLUE_BASKET_CLOSE = new Vector2D(constants.VIRT_BLUE_BASKET_X, constants.VIRT_BLUE_BASKET_Y);
    public static final Vector2D RED_BASKET_CLOSE = new Vector2D(constants.VIRT_RED_BASKET_X, constants.VIRT_RED_BASKET_Y);
    public static final Vector2D RED_BASKET_FAR = new Vector2D(constants.VIRT_RED_BASKET_FAR_X, constants.VIRT_RED_BASKET_FAR_Y);
    public static final Vector2D BLUE_BASKET_FAR = new Vector2D(constants.VIRT_BLUE_BASKET_FAR_X, constants.VIRT_BLUE_BASKET_FAR_Y);
    public static final Vector2D NO_POSITION = new Vector2D(0, 15);

    private final double WHEEL_RADIUS_INCHES = constants.VGS_WHEEL_RADIUS;
    private final double GEAR_RATIO = constants.VGS_GEAR_RATIO;
    private final double HOOD_MIN_ANGLE = constants.VGS_HOOD_MIN_ANGLE;
    private final double HOOD_MAX_ANGLE = constants.VGS_HOOD_MAX_ANGLE;

    private final double TURRET_MAX_DEG = constants.TURRET_MAX_DEG;
    private final double TURRET_MIN_DEG = constants.TURRET_MIN_DEG;
    private final double TICKS_PER_DEGREE = constants.TURRET_TICKS_PER_DEGREE;
    private final double TICKS_PER_REV_SHOOTER = constants.SHOOTER_COUNTS_PER_MOTOR_REV;

    private double baseP = 0.1;
    private double baseI = 0.0;
    private double baseD = 0.0;
    private double baseF = 0.0;

    // Rate limiter and power limits
    private double maxPower = 0.7;
    private double maxDeltaPower = 0.03;
    private double lastOutput = 0;
    private double turretDeadband = 5.0; // degrees

    public VirtualGoalShooter.StartPosition currentStartState = VirtualGoalShooter.StartPosition.NO_POSITION;

    private double normalP = constants.SHOOTER_PID_NORMAL_P, boostP = constants.SHOOTER_PID_BOOST_P, rpmDropThreshold = constants.SHOOTER_RPM_DROP_THRESHOLD;

    private DcMotorEx turretMotor, rightShooter;
    private Servo leftHood, rightHood;
    private Servo rgbIndicator;
    private SparkFunOTOS otos;

    private Vector2D targetPos = BLUE_BASKET_FAR;
    private InterpLUT rpmTable = new InterpLUT("RPM");
    private InterpLUT hoodTable = new InterpLUT("Hood");

    private PIDController turretPID;
    private KalmanFilter turretFilter;

    public enum TurretState { TRACKING, UNWINDING }
    private TurretState currentState = TurretState.TRACKING;
    private double unwindTargetAngle = 0;

    // Cooldown to prevent unwind → tracking oscillation loop
    private int unwindCooldownCycles = 0;
    private static final int UNWIND_COOLDOWN = constants.VGS_UNWIND_COOLDOWN;

    private static final double SOFT_LIMIT_ZONE = constants.VGS_SOFT_LIMIT_ZONE;
    private static final double WALL_PROTECTION_ZONE = constants.VGS_WALL_PROTECTION_ZONE;

    private boolean flywheelEnabled = false;
    private double targetRPM = 0;

    private FiringSolution lastSolution = null;

    public void init(HardwareMap hardwareMap, SparkFunOTOS otosRef) {
        this.otos = otosRef;

        turretPID = new PIDController(baseP, baseI, baseD, baseF);
        turretPID.setMaxIntegral(constants.VGS_TURRET_MAX_INTEGRAL);

        // Kalman filter for turret encoder — smooths out flywheel vibration noise
        // Q = process noise (low = turret position changes slowly)
        // R = measurement noise (higher = more vibration rejection)
        turretFilter = new KalmanFilter(constants.VGS_TURRET_KALMAN_Q, constants.VGS_TURRET_KALMAN_R);

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

        // Gobilda RGB Indicator (PWM — controlled as a servo)
        rgbIndicator = hardwareMap.get(Servo.class, "rgbIndicator");

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

    public void switchAlliance(boolean blue, boolean far, boolean noPosition) {
        if (blue && !far && !noPosition) {
            currentStartState = VirtualGoalShooter.StartPosition.CLOSE_BLUE;
        } else if (blue && far && !noPosition) {
            currentStartState = VirtualGoalShooter.StartPosition.FAR_BLUE;
        } else if (!blue && !far && !noPosition) {
            currentStartState = VirtualGoalShooter.StartPosition.CLOSE_RED;
        } else if (!blue && far && !noPosition) {
            currentStartState = VirtualGoalShooter.StartPosition.FAR_RED;
        } else {
            currentStartState = VirtualGoalShooter.StartPosition.NO_POSITION;
        }
    }

    public void updateShooter() {
        switch (currentStartState) {
            case CLOSE_BLUE:
                targetPos = BLUE_BASKET_CLOSE;
                break;

            case CLOSE_RED:
                targetPos = RED_BASKET_CLOSE;
                break;

            case FAR_BLUE:
                targetPos = BLUE_BASKET_FAR;
                break;

            case FAR_RED:
                targetPos = RED_BASKET_FAR;
                break;

            case NO_POSITION:
                targetPos = NO_POSITION;
                break;
        }
    }

    /**
     * Call this in your loop - it automatically tracks the turret and hood
     * but does NOT spin the flywheel unless spinUpShooter() has been called
     */
    public void update() {
        // Sync targetPos with the current alliance selection
        updateShooter();

        updateShooterPIDF();

        updateLED();

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

    // Gobilda RGB Indicator color positions (PWM servo values)
    private static final double LED_GREEN = 0.39;
    private static final double LED_RED = 0.0;
    private static final double LED_OFF = 0.5;  // mid-range = off/white

    /**
     * Updates the Gobilda RGB Indicator
     * Green = shooter at RPM and ready to fire
     * Red = not ready
     */
    private void updateLED() {
        if (flywheelEnabled) {
            rgbIndicator.setPosition(isReadyToShoot() ? LED_GREEN : LED_RED);
        } else {
            rgbIndicator.setPosition(LED_OFF);
        }
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

        double dx = 0 - pos.x; // -30
        double dy = 100 - pos.y; // 100
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

        double targetFieldAngle = -Math.toDegrees(Math.atan2(dx, dy));
        double relativeTurretAngle = targetFieldAngle - chassisHeading;



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

    /**
     * Picks the best reachable angle (trying target and target ± 360°),
     * applies soft limits near mechanical stops, and protects against wall impact.
     */
    private void moveTurretToAngle(double targetAngle) {
        double currentAngle = getTurretDegrees();

        // Always evaluate the target AND target ± 360° to find the best in-range option
        targetAngle = pickBestAngle(targetAngle, currentAngle);

        // If even the best candidate is out of range, we need to unwind
        if (targetAngle > TURRET_MAX_DEG || targetAngle < TURRET_MIN_DEG) {
            // Only unwind if cooldown has expired (prevents oscillation loop)
            if (unwindCooldownCycles <= 0) {
                double altAngle = (targetAngle > 0) ? targetAngle - 360 : targetAngle + 360;
                initiateUnwind(altAngle);
                return;
            }
            // During cooldown, clamp to nearest safe limit instead of unwinding
            targetAngle = LinearMath.clamp(targetAngle, TURRET_MIN_DEG + WALL_PROTECTION_ZONE,
                    TURRET_MAX_DEG - WALL_PROTECTION_ZONE);
        }

        // Decrement cooldown
        if (unwindCooldownCycles > 0) unwindCooldownCycles--;

        // Deadband: stop motor when close enough (prevents jitter)
        double error = targetAngle - currentAngle;
        if (Math.abs(error) < turretDeadband) {
            turretMotor.setPower(0);
            lastOutput = 0;
            return;
        }

        // Adaptive PID Gains — matched from ShooterSubsystem
        if (Math.abs(error) > 20) {
            turretPID.setPIDF(baseP * 1.1, baseI, baseD, baseF);  // 4x normal
        } else if (Math.abs(error) > 10) {
            turretPID.setPIDF(baseP, baseI, baseD, baseF);  // 2x normal
        } else {
            turretPID.setPIDF(baseP, baseI, baseD, baseF);
        }

        double power = turretPID.update(targetAngle, currentAngle);

        // Rate limiter: prevent sudden power jumps for smoother motion
        double delta = power - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            power = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = power;

        // Clamp to max power
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // Soft Limit: Fade power near mechanical stops
        power = applySoftLimits(power, currentAngle);

        turretMotor.setPower(power);
    }

    /**
     * Evaluates target, target+360, and target-360 to find the angle that is
     * within mechanical limits and closest to the current position.
     */
    private double pickBestAngle(double target, double current) {
        double[] candidates = { target, target + 360, target - 360 };
        double bestAngle = target;
        double bestDist = Double.MAX_VALUE;

        for (double candidate : candidates) {
            // Only consider candidates within mechanical range
            if (candidate >= TURRET_MIN_DEG && candidate <= TURRET_MAX_DEG) {
                double dist = Math.abs(candidate - current);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestAngle = candidate;
                }
            }
        }

        // If no candidate is in range, return original (will trigger unwind)
        return bestAngle;
    }

    /**
     * Reduces power near mechanical limits and zeros it if pushing into a wall.
     */
    private double applySoftLimits(double power, double currentAngle) {
        double distToMax = TURRET_MAX_DEG - currentAngle;
        double distToMin = currentAngle - TURRET_MIN_DEG;

        // Wall protection: zero power if at a limit and pushing into it
        if (distToMax < WALL_PROTECTION_ZONE && power > 0) return 0;
        if (distToMin < WALL_PROTECTION_ZONE && power < 0) return 0;

        // Soft limit: linearly scale power down as we approach the edge
        if (distToMax < SOFT_LIMIT_ZONE && power > 0) {
            double scale = distToMax / SOFT_LIMIT_ZONE;
            power *= scale;
        }
        if (distToMin < SOFT_LIMIT_ZONE && power < 0) {
            double scale = distToMin / SOFT_LIMIT_ZONE;
            power *= scale;
        }

        return power;
    }

    private void initiateUnwind(double target) {
        currentState = TurretState.UNWINDING;
        unwindTargetAngle = LinearMath.clamp(target, TURRET_MIN_DEG + 7.5, TURRET_MAX_DEG - 7.5);
        turretPID.reset();
    }

    private void executeUnwind() {
        // Slightly higher P for unwind, no I/D — just get there smoothly
        turretPID.setPIDF(baseP * 1.5, 0, 0, 0);

        double power = turretPID.update(unwindTargetAngle, getTurretDegrees());

        // Rate limiter: same as tracking
        double delta = power - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            power = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = power;

        // Clamp to max power
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // Apply same soft limits during unwind
        power = applySoftLimits(power, getTurretDegrees());

        turretMotor.setPower(power);

        if (Math.abs(unwindTargetAngle - getTurretDegrees()) < 5.0) {
            currentState = TurretState.TRACKING;
            unwindCooldownCycles = UNWIND_COOLDOWN; // Prevent immediate re-unwind
            lastOutput = 0;
            turretPID.reset();
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
        double raw = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
        return turretFilter.filter(raw);
    }

    /** Raw unfiltered turret angle — useful for telemetry/debugging */
    public double getTurretDegreesRaw() {
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

    public double getHoodAngle() {
        return lastSolution != null ? lastSolution.hoodAngle : 0;
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