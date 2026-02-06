package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSubsystem {

    // 4200 + max hood angle for far zone shot

    /*
    CLOSE TRIPLE
    0.65 hood
    3300 rpm
    38.5 inches

    MEDIUM TRIPLE
    0.45 hood
    3400 rpm
    56 inches

    LONG TRIPLE
    0.45 hood
    3700 rpm
    85 inches

    FAR ZONE
    x hood :)
    4600 rpm
     */

    /*
    constant 0.45 hood
    38.5 in -> 3400
    43.5 in -> 3400
    48.5 in -> 3300
    53.5 in -> 3300
    58.5 in -> 3300
    63.5 in -> 3400
    68.5 in -> 3450
    73.5 in -> 3500
    78.5 in -> 3650
    83.5 in -> 3700

    37 in -> 0.45 3100
    42 -> 0.45 3100
    47 -> 0.45 3200
    52 -> 0.45 3200
    57 -> 0.45 3200
    62 -> 0.45 3300
    67 -> 0.35 3400
    72 -> 0.15 3600

    129 in (far zone) -> 0, 4300

    f(x) = 0.001(x-53)^4 + 3300
     */

    public enum ShootState {
        FAR_LOB_SHOT,
        FAR_HARD_SHOT,
        MEDIUM_SHOT,
        CLOSE_SHOT,
        NO_SHOT,
    }

    public enum TurretState {
        TRACKING,
        UNWINDING
    }

    private AprilTagLimelight limeLight = new AprilTagLimelight();
    private DcMotorEx turretMotor = null;
    private DcMotorEx rightShooter = null;
    private Servo leftHood = null;
    private Servo rightHood = null;
//    private Servo ledLight = null;
    private SparkFunOTOS otos;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime dt = new ElapsedTime();

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double kP = 0.002; // 0.002
    private double kI = 0.0001; // 0.0001
    private double kD = 0.00; // 0.00

    private double hoodPosition = 0.2;
    private double filterStrength = 0.8;
    private double deadband = 4.5;
    private double maxPower = 0.7;
    private double maxDeltaPower = 0.03;
    private double turretMinSpeed = 0.1;
    private double turretManualSpeed = 120; // deg/s
    private double lastFilteredTx = 0;
    private double lastOutput = 0;

    private final double TICKS_PER_REV = 145.1;
    private final double GEAR_RATIO = 5.0;
    private final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
    private final int TURRET_MAX_TICKS = (int)(235 * TICKS_PER_DEGREE);
    private final int TURRET_MIN_TICKS = (int)(-125 * TICKS_PER_DEGREE);

    private final double COUNTS_PER_MOTOR_REV = 28.0;
    private final double GEAR_REDUCTION = 1;
    private final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;

    private ShootState currentHoodState = ShootState.NO_SHOT;
    private double targetRPM = 0;
    private double alternativeAngle = 0;

    private TurretState currentState = TurretState.TRACKING;
    private double unwindTargetAngle = 0;
    private double unwindStartHeading = 0;

    // Debug variables for telemetry gng
    private double lastLimelightYaw = 0;
    private double lastCalculatedTargetAngle = 0;
    private double lastTX = 0;

    // Hybrid calibration mode
    private boolean calibrationMode = true;  // Start in reactive mode (visual tracking)
    private boolean wantsToCalibrate = false;  // True when Y is pressed to trigger calibration yayay
    private double savedOtosHeading = 0;   // OTOS heading when calibrated
    private double savedTurretAngle = 0;   // Turret angle when calibrated
    private boolean offsetCalibrated = false;
    private int targetAprilTagIdBLUE = 20;
    private int targetAprilTagIdRED = 24;
    private double searchAngle = 0;  // Default angle to search for AprilTag when lost

    // Field-centric tracking - target tower coordinates (tune these!)
    private double targetTowerX = 0.0;   // X coordinate of blue tower in inches (from OTOS origin)
    private double targetTowerY = 0.0;    // Y coordinate of blue tower in inches

    private double turretAutoSpeed = 0.4;

    private int tgtAprilTagId = 0;

    public void init(HardwareMap hardwareMap, SparkFunOTOS otosRef) {
        initSystem(hardwareMap, otosRef);
    }

    public void init(HardwareMap hardwareMap, SparkFunOTOS otosRef, int targetAprilTagId) {
        this.tgtAprilTagId = targetAprilTagId;
        initSystem(hardwareMap, otosRef);
    }

    private void initSystem(HardwareMap hardwareMap, SparkFunOTOS otosRef)
    {
        limeLight.init(hardwareMap);
        this.otos = otosRef;
//        ledLight = hardwareMap.get(Servo.class, "RPM_Light");

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor"); // Unchanged
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(27,0,0,0));
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter"); //
        rightShooter.setDirection(DcMotorEx.Direction.FORWARD);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(100,1,0,0));

        leftHood = hardwareMap.get(Servo.class, "leftHood"); //
        leftHood.setDirection(Servo.Direction.REVERSE);
        rightHood = hardwareMap.get(Servo.class, "rightHood"); //

        timer.reset();
    }
    
    // Nathans request
    private double normalP = 100;
    private double boostP = 200;
    private double rpmDropThreshold = 300;
    
    public void updateShooterPIDF() {
        double error = getTargetRPM() - getCurrentRPM();
        if (error > rpmDropThreshold && getTargetRPM() > 0) {
            rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(boostP, 0, 1, 1));
        } else {
            rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(normalP, 0, 1, 1));
        }
    }

    private final double limelightDistConst = 3;

    // rpm as a function of x
    private double r(double x)
    {
        if (x == 0) {return 0;} // for if it doesnt see anything stop the motah

        return -0.0002 * Math.pow(x,5) + 0.0483 * Math.pow(x,4) -
                5.2488 * Math.pow(x,3) + 280.38 * Math.pow(x,2) - 7354.3 * x
                + 78852;
    }

    // hood as a function of x
    private double h(double x)
    {
        // restrictions YAY
        if (x == 0) { return 0; }
        if (x > 72) { return 0.15; }

        return Math.pow(9,-9) * Math.pow(x,6) -
                Math.pow(3,-6) * Math.pow(x,5) +
                0.0004 * Math.pow(x,4) - 0.0271 * Math.pow(x,3) +
                1.056 * Math.pow(x,2) - 21.712 * x + 184.28;
    }

    private double bangBangCoef = 1.2;


    private final double rpmLenience = 100;
    public boolean IsAtTgtRPM()
    {
        return Math.abs(targetRPM - getCurrentRPM()) < rpmLenience;
    }

    public void BangBang() {
        double error = getTargetRPM() - getCurrentRPM();
        double ticksPerSecond = (getTargetRPM() / 60.0) * COUNTS_PER_WHEEL_REV;
        if (error > 0) {
            rightShooter.setVelocity(ticksPerSecond * bangBangCoef);
        } else {
            rightShooter.setVelocity(ticksPerSecond);
        }
    }

    ElapsedTime hoodResetTimer = new ElapsedTime();
    int hoodResetHz = 3;

    public void update() {
        double currentDistance = limeLight.GetDistance();
        final double CLOSE_LIMIT = 38.5;
        final double MEDIUM_LIMIT = 56;

        if (currentDistance > 0 && currentDistance < CLOSE_LIMIT) {
            currentHoodState = ShootState.CLOSE_SHOT;
        } else if (currentDistance >= CLOSE_LIMIT && currentDistance < MEDIUM_LIMIT) {
            currentHoodState = ShootState.MEDIUM_SHOT;
        } else if (currentDistance >= MEDIUM_LIMIT) {
            currentHoodState = ShootState.FAR_HARD_SHOT;
        } else {
            currentHoodState = ShootState.NO_SHOT;
        }

        // cant shoot...
        if (currentDistance < 20 || currentDistance > 90)
        {
            setHoodPosition(0.45);
            setTargetRPM(0);

        }
        else
        {
            // double curve yay

            if (hoodResetTimer.seconds() > 1.0 / hoodResetHz)
            {
                setHoodPosition(h(currentDistance));
                hoodResetTimer.reset();
            }
            setTargetRPM(-(int)(r(currentDistance)));
        }
//        updateHood();
    }

    ElapsedTime dx = new ElapsedTime();

    public void decideManualOrTxBLUE(double input) {
        // Zephyr had noted a confliction with turnToAngle and unwinding with manual
        if (currentState == TurretState.UNWINDING) {
            AggresiveTxTracking();
            dt.reset();
            return;
        }

        if (limeLight.GetLimelightId() != targetAprilTagIdBLUE) {
            // Tag lost: Use manual control
            turnToAngle(getTurretAngle() + (input * dt.seconds() * turretManualSpeed), false);
        } else {
            // Tag seen: Track it and pack it!!
            trackTargetHybrid();
        }

        dt.reset();
    }

    public void decideManualOrTxRED(double input)
    {
        if (limeLight.GetLimelightId() != targetAprilTagIdRED)
        {
            turnToAngle(getTurretAngle() + (input * dt.seconds() * turretManualSpeed), false);
        }
        else
        {
//            AggresiveTxTracking();
        }

        dt.reset();
    }

    public void AggresiveTxTracking()
    {
        kP = 0.01;
        kI = 0.0;
        kD = 0.0;

        if (currentState == TurretState.UNWINDING) {
            double currentHeading = Math.toDegrees(otos.getPosition().h);
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;

            turnToAngle(dynamicTarget, false);

            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;
        }

        double tx = limeLight.GetTX();

//        int id = limeLight.GetLimelightId();
//        if (id != 0)
//        {
//            if (limeLight.GetLimelightId() != targetAprilTagId) {
//                turretMotor.setPower(0);
//                integralSum = 0;
//                lastError = 0;
//                timer.reset();
//                return;
//            }
//        }

        double dt = timer.seconds();
        if (dt <= 0) dt = 0.001;

        double filteredTx = tx * 0.2 + lastFilteredTx * 0.8;
        lastFilteredTx = filteredTx;

        if (Math.abs(tx) < deadband) {
            turretMotor.setPower(0);
            lastOutput = 0;
            integralSum = 0;
            return;
        }

        double error = 0-filteredTx;
        double derivative = (error - lastError) / dt;
        lastError = error;
        integralSum += error * dt;

        double output = kP * error + kI * integralSum + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;
        timer.reset();

        output = Math.max(-maxPower, Math.min(maxPower, output));

        int currentPos = turretMotor.getCurrentPosition();
        boolean hitMax = (currentPos > TURRET_MAX_TICKS && output > 0);
        boolean hitMin = (currentPos < TURRET_MIN_TICKS && output > 0);

        if (hitMax || hitMin) {
            turretMotor.setPower(0);

            double currentAngle = getTurretAngle();
            alternativeAngle = hitMax ? (currentAngle - 360) : (currentAngle + 360);

            double altTicks = alternativeAngle * TICKS_PER_DEGREE;
            if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                currentState = TurretState.UNWINDING;
                unwindTargetAngle = alternativeAngle;
                unwindStartHeading = Math.toDegrees(otos.getPosition().h);
            }
            return;
        }

        if (output > 0) output += turretMinSpeed;
        if (output < 0) output -= turretMinSpeed;

        turretMotor.setPower(output * turretAutoSpeed);
    }

    // converted to blue and red, passed into the init in comp op
    public void trackTargetHybrid() {
        // Handle unwinding first
        if (currentState == TurretState.UNWINDING) {
            double currentHeading = Math.toDegrees(otos.getPosition().h);
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;
            turnToAngle(dynamicTarget, false);
            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;
        }

        // No AprilTag? If calibrated, use OTOS prediction. If not, stop.
        if (limeLight.GetLimelightId() != tgtAprilTagId) {
            if (offsetCalibrated) {
                // PREDICTIVE: Use OTOS heading to track even without vision
                double currentHeading = Math.toDegrees(-otos.getPosition().h); // flipping for coordinate systems
                double headingChange = currentHeading - savedOtosHeading;
                // Counter-rotate turret as robot rotates
                double targetAngle = savedTurretAngle - headingChange;
                // Normalize
                while (targetAngle > 180) targetAngle -= 360;
                while (targetAngle < -180) targetAngle += 360;

                // Turret limit check
                double targetTicks = targetAngle * TICKS_PER_DEGREE;
                int currentPos = turretMotor.getCurrentPosition();
                if (targetTicks > TURRET_MAX_TICKS || targetTicks < TURRET_MIN_TICKS) {
                    if (currentPos > TURRET_MAX_TICKS || currentPos < TURRET_MIN_TICKS) {
                        turretMotor.setPower(0);
                        double currentAngle = getTurretAngle();
                        alternativeAngle = (currentPos > TURRET_MAX_TICKS) ? (currentAngle - 360) : (currentAngle + 360);
                        double altTicks = alternativeAngle * TICKS_PER_DEGREE;
                        if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                            currentState = TurretState.UNWINDING;
                            unwindTargetAngle = alternativeAngle;
                            unwindStartHeading = currentHeading;
                        }
                    }
                    return;
                }
                turnToAngle(targetAngle, false);
            } else {
                turretMotor.setPower(0);
            }
            integralSum = 0;
            lastError = 0;
            return;
        }

        double tx = limeLight.GetTX();
        lastTX = tx;
        double filteredTx = filterStrength * lastFilteredTx + (1 - filterStrength) * tx;
        lastFilteredTx = filteredTx;

        // CALIBRATE: When centered on AprilTag, save OTOS heading + turret angle
        if (Math.abs(filteredTx) < deadband && !offsetCalibrated) {
            savedOtosHeading = Math.toDegrees(-otos.getPosition().h);
            savedTurretAngle = getTurretAngle();
            offsetCalibrated = true;
        }

        // OTOS heading based PID
        if (offsetCalibrated) {
            double currentHeading = Math.toDegrees(-otos.getPosition().h);
            double headingChange = currentHeading - savedOtosHeading;
            double targetAngle = savedTurretAngle - headingChange;

            // Normalize
            while (targetAngle > 180) targetAngle -= 360;
            while (targetAngle < -180) targetAngle += 360;

            lastCalculatedTargetAngle = targetAngle;

            // Slow drift correction using tx
//            if (Math.abs(tx) > 0.5) {
//                savedTurretAngle += tx * 0.005;
//            }

            // Turret limit check
            double targetTicks = targetAngle * TICKS_PER_DEGREE;
            int currentPos = turretMotor.getCurrentPosition();
            if (targetTicks > TURRET_MAX_TICKS || targetTicks < TURRET_MIN_TICKS) {
                if (currentPos > TURRET_MAX_TICKS || currentPos < TURRET_MIN_TICKS) {
                    turretMotor.setPower(0);
                    double currentAngle = getTurretAngle();
                    alternativeAngle = (currentPos > TURRET_MAX_TICKS) ? (currentAngle - 360) : (currentAngle + 360);
                    double altTicks = alternativeAngle * TICKS_PER_DEGREE;
                    if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                        currentState = TurretState.UNWINDING;
                        unwindTargetAngle = alternativeAngle;
                        unwindStartHeading = currentHeading;
                    }
                }
                return;
            }

            turnToAngle(targetAngle, false);
            return;
        }

        // tx based PID (reactive mode before calibration)
        double dt = timer.seconds();
        if (dt <= 0) dt = 0.001;
        double error = filteredTx;

        if (Math.abs(error) < deadband) {
            turretMotor.setPower(0);
            lastOutput = 0;
            integralSum = 0;
            timer.reset();
            return;
        }

        double derivative = (error - lastError) / dt;
        lastError = error;
        integralSum += error * dt;
        double output = kP * error + kI * integralSum + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;
        timer.reset();
        output = Math.max(-maxPower, Math.min(maxPower, output));

        int currentPos = turretMotor.getCurrentPosition();
        boolean hitMax = (currentPos > TURRET_MAX_TICKS && -output > 0);
        boolean hitMin = (currentPos < TURRET_MIN_TICKS && -output < 0);

        if (hitMax || hitMin) {
            turretMotor.setPower(0);
            double currentAngle = getTurretAngle();
            alternativeAngle = hitMax ? (currentAngle - 360) : (currentAngle + 360);
            double altTicks = alternativeAngle * TICKS_PER_DEGREE;
            if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                currentState = TurretState.UNWINDING;
                unwindTargetAngle = alternativeAngle;
                unwindStartHeading = Math.toDegrees(otos.getPosition().h);
            }
            return;
        }

        if (output > 0) output += turretMinSpeed;
        if (output < 0) output -= turretMinSpeed;
        turretMotor.setPower(-output);
    }

    public void requestCalibration() {
        wantsToCalibrate = true;  // Request calibration on next center
    }

    public boolean isCalibrated() {
        return offsetCalibrated;
    }

    public double getSavedOffset() {
        return savedOtosHeading;
    }

    public void setSearchAngle(double angle) {
        this.searchAngle = angle;
    }

    public void setTargetAprilTagId(int aprilTagId) {
        this.targetAprilTagIdBLUE = aprilTagId;
    }

    public void setTargetTowerPosition(double x, double y) {
        this.targetTowerX = x;
        this.targetTowerY = y;
    }

    /**
     * FIELD-CENTRIC TRACKING for da tower position
     * Uses atan2 to calculate angle from robot position to target tower
     * Works regardless of where robot moves on field so it's better than otos
     * called manually btw
     */
    public void trackTargetFieldCentric() {
        // Handle unwinding first
        if (currentState == TurretState.UNWINDING) {
            double currentHeading = Math.toDegrees(otos.getPosition().h);
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;
            turnToAngle(dynamicTarget, false);
            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;
        }

        // Get robot position from OTOS
        double robotX = otos.getPosition().x;
        double robotY = otos.getPosition().y;
        double robotHeading = Math.toDegrees(otos.getPosition().h);

        // Calculate angle to target tower using atan2
        double dx = targetTowerX - robotX;
        double dy = targetTowerY - robotY;
        double fieldAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));

        // Convert field angle to turret angle (relative to robot heading)
        double turretTargetAngle = fieldAngleToTarget - robotHeading;

        // Normalize to ±180
        while (turretTargetAngle > 180) turretTargetAngle -= 360;
        while (turretTargetAngle < -180) turretTargetAngle += 360;

        lastCalculatedTargetAngle = turretTargetAngle;

        // If we see the AprilTag, use tx for fine correction
        if (limeLight.GetLimelightId() == targetAprilTagIdBLUE) {
            double tx = limeLight.GetTX();
            lastTX = tx;
            // Blend vision correction with odometry
            turretTargetAngle = turretTargetAngle * 0.7 + (getTurretAngle() - tx) * 0.3;
        }

        // Turret limit check
        double targetTicks = turretTargetAngle * TICKS_PER_DEGREE;
        int currentPos = turretMotor.getCurrentPosition();

        if (targetTicks > TURRET_MAX_TICKS || targetTicks < TURRET_MIN_TICKS) {
            if (currentPos > TURRET_MAX_TICKS || currentPos < TURRET_MIN_TICKS) {
                turretMotor.setPower(0);
                double currentAngle = getTurretAngle();
                alternativeAngle = (currentPos > TURRET_MAX_TICKS) ? (currentAngle - 360) : (currentAngle + 360);
                double altTicks = alternativeAngle * TICKS_PER_DEGREE;
                if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                    currentState = TurretState.UNWINDING;
                    unwindTargetAngle = alternativeAngle;
                    unwindStartHeading = robotHeading;
                }
            }
            return;
        }

        turnToAngle(turretTargetAngle, false);
    }

    public void trackTargetPredictive(double targetOffsetDegrees) {
        // Handle unwinding state cause thats more important
        if (currentState == TurretState.UNWINDING) {
            double currentHeading = Math.toDegrees(otos.getPosition().h);
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;
            turnToAngle(dynamicTarget, false);
            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;
        }

        if (limeLight.GetLimelightId() != 21 ) {
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = 0;
            timer.reset();
            return;
        }

        // Get Limelight's absolute... orientation from IMU
        double limelightYaw = limeLight.GetYaw();
        lastLimelightYaw = limelightYaw;  // Store for debug

        // Target angle = Limelight yaw + offset for target position
        double targetAngle = limelightYaw + targetOffsetDegrees;
        lastCalculatedTargetAngle = targetAngle;  // Store for debug

        // Normalize to 180 deg
        while (targetAngle > 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;

        // Check if target is within turet limmits
        double targetTicks = targetAngle * TICKS_PER_DEGREE;
        int currentPos = turretMotor.getCurrentPosition();

        if (targetTicks > TURRET_MAX_TICKS || targetTicks < TURRET_MIN_TICKS) {
            // Target out of range? NOT ANYMORE!!
            if (currentPos > TURRET_MAX_TICKS || currentPos < TURRET_MIN_TICKS) {
                turretMotor.setPower(0);
                double currentAngle = getTurretAngle();
                alternativeAngle = (currentPos > TURRET_MAX_TICKS) ? (currentAngle - 360) : (currentAngle + 360);
                double altTicks = alternativeAngle * TICKS_PER_DEGREE;
                if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                    currentState = TurretState.UNWINDING;
                    unwindTargetAngle = alternativeAngle;
                    unwindStartHeading = Math.toDegrees(otos.getPosition().h);
                }
            }
            return;
        }

        // Use PID to smoothly reachd a angle
        turnToAngle(targetAngle, false);
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
        double ticksPerSecond = (targetRPM / 60.0) * COUNTS_PER_WHEEL_REV;
//        rightShooter.setVelocity(ticksPerSecond);
    }
    public double targetTurretAngle = 0;


    public void stopTurret()
    {
        turretMotor.setPower(0);
    }

    public void turnToAngle(double targetAngle, boolean negateOutput) {
        double currentAngle = getTurretAngle();
        double error = targetAngle - currentAngle;
        targetTurretAngle = targetAngle;

        if (Math.abs(error) < 5)
        {
            turretMotor.setPower(0);
            return;
        }

        turretMotor.setTargetPosition((int)(targetAngle * TICKS_PER_DEGREE));

        double dtSeconds = timer.seconds();
        timer.reset();
        if (dtSeconds <= 0) dtSeconds = 0.001;

        integralSum += error * dtSeconds;
        double maxIntegral = 1.0;
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));

        double derivative = (error - lastError) / dtSeconds;
        lastError = error;

        // ADAPTIVE GAINS: Just like my awesome muscles
        double adaptiveKP = kP;
        if (Math.abs(error) > 20) {
            adaptiveKP = 0.008;  // 4x normal gain
        } else if (Math.abs(error) > 10) {
            adaptiveKP = 0.004;  // 2x normal gain
        }

        double output = adaptiveKP * error + kI * integralSum + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;

        output = Math.max(-maxPower, Math.min(maxPower, output));

        if (negateOutput) {
            output = -output;
        }

        // prevent integral windup by resetting when pass over
        if (output * lastOutput < 0)
        {
            integralSum = 0;
        }

        int currentPos = turretMotor.getCurrentPosition();
        boolean atMaxGoingFurther = (currentPos > TURRET_MAX_TICKS && output > 0);
        boolean atMinGoingFurther = (currentPos < TURRET_MIN_TICKS && output < 0);

        if (atMaxGoingFurther || atMinGoingFurther) {
            turretMotor.setPower(0);
            return;
        }

        turretMotor.setPower(output);
    }

    private void updateHood() {
        switch(currentHoodState) {
//            case FAR_LOB_SHOT:
//                setHoodPosition(0.40);
//                setTargetRPM(3500); // not actual speed, rn the logic is broken
//                break;
            case FAR_HARD_SHOT:
                setHoodPosition(0.45);
                setTargetRPM(3700);
                break;
            case MEDIUM_SHOT:
                setHoodPosition(0.45);
                setTargetRPM(3400);
                break;
            case CLOSE_SHOT:
                setHoodPosition(0.65);
                setTargetRPM(3300);
                break;
            case NO_SHOT:
                setHoodPosition(0.30);
                break;
        }
    }


    private final double turretLenience = 5; // degrees

    public boolean IsAtCorrectTurretPos()
    {
        return Math.abs(targetTurretAngle - getTurretAngle()) < turretLenience;
    }

    public void setHoodPosition(double value) {
        leftHood.setPosition(value);
        rightHood.setPosition(value);
    }

    // Public getters for telemetry
    public double getTargetRPM() { return targetRPM; }

    public double getDistance() { return limeLight.GetDistance();}
    public double getCurrentRPM() {
        return (rightShooter.getVelocity() / COUNTS_PER_WHEEL_REV) * 60.0;
    }
    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }
    public int getTurretTicks() { return turretMotor.getCurrentPosition(); }
    public TurretState getTurretState() { return currentState; }
    public double getLimelightYaw() { return lastLimelightYaw; }
    public double getLastTargetAngle() { return lastCalculatedTargetAngle; }
    public boolean isCalibrationMode() { return calibrationMode; }
    public double getLastTX() { return lastTX; }
    public double getDeadband() { return deadband; }
}

// hi liam :)