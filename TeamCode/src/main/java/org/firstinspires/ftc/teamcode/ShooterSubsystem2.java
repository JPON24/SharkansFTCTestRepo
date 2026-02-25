//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class ShooterSubsystem2 {
//    public enum ShootState {
//        FAR_LOB_SHOT,
//        FAR_HARD_SHOT,
//        MEDIUM_SHOT,
//        CLOSE_SHOT,
//        NO_SHOT,
//    }
//
//    public enum TurretState {
//        TRACKING,
//        UNWINDING
//    }
//
//    private AprilTagLimelight limeLight = new AprilTagLimelight();
//    private DcMotorEx turretMotor = null;
//    private DcMotorEx rightShooter = null;
//    private Servo leftHood = null;
//    private Servo rightHood = null;
//    //    private Servo ledLight = null;
//    private SparkFunOTOS otos;
//
//    private ElapsedTime timer = new ElapsedTime();
//    private ElapsedTime dt = new ElapsedTime();
//
//    private double integralSum = 0.0;
//    private double lastError = 0.0;
//    private double kP = 0.002; // 0.002
//    private double kI = 0.0001; // 0.0001
//    private double kD = 0.00; // 0.00
//
//    private double hoodPosition = 0.2;
//    private double filterStrength = 0.8;
//    private double deadband = 4.5;
//    private double maxPower = 0.7;
//    private double maxDeltaPower = 0.03;
//    private double turretMinSpeed = 0.1;
//    private double turretManualSpeed = 120; // deg/s
//    private double lastFilteredTx = 0;
//    private double lastOutput = 0;
//
//    private final double TICKS_PER_REV = 145.1;
//    private final double GEAR_RATIO = 5.0;
//    private final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
//    private final int TURRET_MAX_TICKS = (int)(235 * TICKS_PER_DEGREE);
//    private final int TURRET_MIN_TICKS = (int)(-125 * TICKS_PER_DEGREE);
//
//    private final double COUNTS_PER_MOTOR_REV = 28.0;
//    private final double GEAR_REDUCTION = 1;
//    private final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
//
//    private ShootState currentHoodState = ShootState.NO_SHOT;
//    private double targetRPM = 0;
//    private double alternativeAngle = 0;
//
//    private TurretState currentState = TurretState.TRACKING;
//    private double unwindTargetAngle = 0;
//    private double unwindStartHeading = 0;
//
//    // Debug variables for telemetry gng
//    private double lastLimelightYaw = 0;
//    private double lastCalculatedTargetAngle = 0;
//    private double lastTX = 0;
//
//    // Hybrid calibration mode
//    private boolean calibrationMode = true;  // Start in reactive mode (visual tracking)
//    private boolean wantsToCalibrate = false;  // True when Y is pressed to trigger calibration yayay
//    private double savedOtosHeading = 0;   // OTOS heading when calibrated
//    private double savedTurretAngle = 0;   // Turret angle when calibrated
//    private boolean offsetCalibrated = false;
//    private int targetAprilTagIdBLUE = 20;
//    private int targetAprilTagIdRED = 24;
//    private double searchAngle = 0;  // Default angle to search for AprilTag when lost
//
//    // Field-centric tracking - target tower coordinates (tune these!)
//    private double targetTowerX = 0.0;   // X coordinate of blue tower in inches (from OTOS origin)
//    private double targetTowerY = 0.0;    // Y coordinate of blue tower in inches
//
//    private double turretAutoSpeed = 0.4;
//
//    private int tgtAprilTagId = 0;
//    private double goalX = 0;
//    private double goalY = 0;
//    private double startHeading = 0;
//    private double startX = 0;
//    private double startY = -3.7;
//
//    public void init(HardwareMap hardwareMap, SparkFunOTOS otosRef) {
//        initSystem(hardwareMap, otosRef);
//    }
//
//    public void init(HardwareMap hardwareMap, SparkFunOTOS otosRef, int targetAprilTagId) {
//        this.tgtAprilTagId = targetAprilTagId;
//        initSystem(hardwareMap, otosRef);
//    }
//
//    private void initSystem(HardwareMap hardwareMap, SparkFunOTOS otosRef)
//    {
//        limeLight.init(hardwareMap);
//        this.otos = otosRef;
////        ledLight = hardwareMap.get(Servo.class, "RPM_Light");
//
//        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor"); // Unchanged
//        turretMotor.setTargetPosition(0);
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(27,0,0,0));
//        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter"); //
//        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);
//        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(100,1,0,0));
//
//        leftHood = hardwareMap.get(Servo.class, "leftHood"); //
//        leftHood.setDirection(Servo.Direction.REVERSE);
//        rightHood = hardwareMap.get(Servo.class, "rightHood"); //
//
//        timer.reset();
//    }
//
//    // Nathans request
//    private double normalP = 100;
//    private double boostP = 200;
//    private double rpmDropThreshold = 300;
//    public void initOtos() {
//        otos.setOffset(new SparkFunOTOS.Pose2D(startX, startY, startHeading));
//    }
//
//
//
//
//
//
//    private double r(double x)
//    {
//        if (x == 0) {return 0;} // for if it doesnt see anything stop the motah
//
//        return -0.0002 * Math.pow(x,5) + 0.0483 * Math.pow(x,4) -
//                5.2488 * Math.pow(x,3) + 280.38 * Math.pow(x,2) - 7354.3 * x
//                + 78852;
//    }
//
//
//    private double h(double x)
//    {
//        // restrictions YAY
//        if (x == 0) { return 0; }
//        if (x > 72) { return 0.15; }
//
//        return Math.pow(9,-9) * Math.pow(x,6) -
//                Math.pow(3,-6) * Math.pow(x,5) +
//                0.0004 * Math.pow(x,4) - 0.0271 * Math.pow(x,3) +
//                1.056 * Math.pow(x,2) - 21.712 * x + 184.28;
//    }
//
//    private double bangBangCoef = 1.2;
//
//
//    private final double rpmLenience = 100;
//
//
//    public void BangBang() {
//        double error = getTargetRPM() - getCurrentRPM();
//        double ticksPerSecond = (getTargetRPM() / 60.0) * COUNTS_PER_WHEEL_REV;
//        if (error > 0) {
//            rightShooter.setVelocity(ticksPerSecond * bangBangCoef);
//        } else {
//            rightShooter.setVelocity(ticksPerSecond);
//        }
//    }
//
//    public void update() {
//        double currentDistance = limeLight.GetDistance();
//        final double CLOSE_LIMIT = 38.5;
//        final double MEDIUM_LIMIT = 56;
//
//        if (currentDistance > 0 && currentDistance < CLOSE_LIMIT) {
//            currentHoodState = ShootState.CLOSE_SHOT;
//        } else if (currentDistance >= CLOSE_LIMIT && currentDistance < MEDIUM_LIMIT) {
//            currentHoodState = ShootState.MEDIUM_SHOT;
//        } else if (currentDistance >= MEDIUM_LIMIT) {
//            currentHoodState = ShootState.FAR_HARD_SHOT;
//        } else {
//            currentHoodState = ShootState.NO_SHOT;
//        }
//
//
//        if (currentDistance == 0)
//        {
//            setHoodPosition(0.45);
//            setTargetRPM(0);
//
//        }
//        else
//        {
//            // double curve yay
//            setHoodPosition(h(currentDistance));
//            setTargetRPM((int)(r(currentDistance)));
//        }
////        updateHood();
//    }
//
//    ElapsedTime dx = new ElapsedTime();
//
//    public void decideManualOrTxBLUE(double input)
//    {
////        turnToAngle(getTurretAngle() + (input * dx.seconds() * turretManualSpeed), false);
//
//        if (limeLight.GetLimelightId() != targetAprilTagIdBLUE)
//        {
//            turnToAngle(getTurretAngle() + (input * dt.seconds() * turretManualSpeed), false);
//        }
//        else
//        {
//            trackTargetHybrid();
//        }
//
//        dt.reset();
//    }
//
//    public void decideManualOrTxRED(double input)
//    {
//        if (limeLight.GetLimelightId() != targetAprilTagIdRED)
//        {
//            turnToAngle(getTurretAngle() + (input * dt.seconds() * turretManualSpeed), false);
//        }
//        else
//        {
////            AggresiveTxTracking();
//        }
//
//        dt.reset();
//    }
//    // converted to blue and red, passed into the init in comp op
//
//    public void manualControl(double input) {
//        turnToAngle(getTurretAngle() + (input * dt.seconds() * turretManualSpeed), false);
//    }
//    public void trackTargetHybrid() {
//
//        if (limeLight.GetLimelightId() != targetAprilTagIdRED) {
//            double currentX = otos.getPosition().x;
//            double currentY = otos.getPosition().y;
//            double currentHeading = otos.getPosition().h;
//            double coolAngle = currentHeading - (90 - Math.atan2(currentY - goalY, currentX - goalX));
//            if (coolAngle > 235) {
//                coolAngle -= 360*(Math.floor((coolAngle-235)/360) + 1);
//            }
//            if (coolAngle < -125) {
//                coolAngle -= 360*(Math.floor((coolAngle+125)/360));
//            }
//            turnToAngle(coolAngle, false);
//
//
//
//
//
//
//
//
//        } else {
//
//            double tx = limeLight.GetTX();
//            lastTX = tx;
//            double filteredTx = filterStrength * lastFilteredTx + (1 - filterStrength) * tx;
//            lastFilteredTx = filteredTx;
//
//            double dt = timer.seconds();
//            if (dt <= 0) dt = 0.001;
//            double error = filteredTx;
//
//            if (Math.abs(error) < deadband) {
//                turretMotor.setPower(0);
//                lastOutput = 0;
//                integralSum = 0;
//                timer.reset();
//                return;
//            }
//
//            double derivative = (error - lastError) / dt;
//            lastError = error;
//            integralSum += error * dt;
//            double output = kP * error + kI * integralSum + kD * derivative;
//
//            double delta = output - lastOutput;
//            if (Math.abs(delta) > maxDeltaPower) {
//                output = lastOutput + Math.signum(delta) * maxDeltaPower;
//            }
//            lastOutput = output;
//            timer.reset();
//            output = Math.max(-maxPower, Math.min(maxPower, output));
//
//            int currentPos = turretMotor.getCurrentPosition();
//            boolean hitMax = (currentPos > TURRET_MAX_TICKS && -output > 0);
//            boolean hitMin = (currentPos < TURRET_MIN_TICKS && -output < 0);
//
//            if (hitMax || hitMin) {
//                turretMotor.setPower(0);
//                return;
//            }
//
//            if (output > 0) output += turretMinSpeed;
//            if (output < 0) output -= turretMinSpeed;
//            turretMotor.setPower(-output);
//        }
//    }
//
//    public void requestCalibration() {
//        wantsToCalibrate = true;  // Request calibration on next center
//    }
//
//    public boolean isCalibrated() {
//        return offsetCalibrated;
//    }
//
//    public double getSavedOffset() {
//        return savedOtosHeading;
//    }
//
//    public void setSearchAngle(double angle) {
//        this.searchAngle = angle;
//    }
//
//    public void setTargetAprilTagId(int aprilTagId) {
//        this.targetAprilTagIdBLUE = aprilTagId;
//    }
//
//    public void setTargetTowerPosition(double x, double y) {
//        this.targetTowerX = x;
//        this.targetTowerY = y;
//    }
//
//    /**
//     * FIELD-CENTRIC TRACKING for da tower position
//     * Uses atan2 to calculate angle from robot position to target tower
//     * Works regardless of where robot moves on field so it's better than otos
//     * called manually btw
//     */
//    public void setTargetRPM(double rpm) {
//        this.targetRPM = rpm;
//        double ticksPerSecond = (targetRPM / 60.0) * COUNTS_PER_WHEEL_REV;
////        rightShooter.setVelocity(ticksPerSecond);
//    }
//    public double targetTurretAngle = 0;
//
//
//    public void stopTurret()
//    {
//        turretMotor.setPower(0);
//    }
//
//    public void turnToAngle(double targetAngle, boolean negateOutput) {
//        double currentAngle = getTurretAngle();
//        double error = targetAngle - currentAngle;
//        targetTurretAngle = targetAngle;
//
//        if (Math.abs(error) < 5)
//        {
//            turretMotor.setPower(0);
//            return;
//        }
//
//        turretMotor.setTargetPosition((int)(targetAngle * TICKS_PER_DEGREE));
//
//        double dtSeconds = timer.seconds();
//        timer.reset();
//        if (dtSeconds <= 0) dtSeconds = 0.001;
//
//        integralSum += error * dtSeconds;
//        double maxIntegral = 1.0;
//        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
//
//        double derivative = (error - lastError) / dtSeconds;
//        lastError = error;
//
//        // ADAPTIVE GAINS: Just like my awesome muscles
//        double adaptiveKP = kP;
//        if (Math.abs(error) > 20) {
//            adaptiveKP = 0.008;  // 4x normal gain
//        } else if (Math.abs(error) > 10) {
//            adaptiveKP = 0.004;  // 2x normal gain
//        }
//
//        double output = adaptiveKP * error + kI * integralSum + kD * derivative;
//
//        double delta = output - lastOutput;
//        if (Math.abs(delta) > maxDeltaPower) {
//            output = lastOutput + Math.signum(delta) * maxDeltaPower;
//        }
//        lastOutput = output;
//
//        output = Math.max(-maxPower, Math.min(maxPower, output));
//
//        if (negateOutput) {
//            output = -output;
//        }
//
//        // prevent integral windup by resetting when pass over
//        if (output * lastOutput < 0)
//        {
//            integralSum = 0;
//        }
//
//        int currentPos = turretMotor.getCurrentPosition();
//        boolean atMaxGoingFurther = (currentPos > TURRET_MAX_TICKS && output > 0);
//        boolean atMinGoingFurther = (currentPos < TURRET_MIN_TICKS && output < 0);
//
//        if (atMaxGoingFurther || atMinGoingFurther) {
//            turretMotor.setPower(0);
//            return;
//        }
//
//        turretMotor.setPower(output);
//    }
//
//    private void updateHood() {
//        switch(currentHoodState) {
////            case FAR_LOB_SHOT:
////                setHoodPosition(0.40);
////                setTargetRPM(3500); // not actual speed, rn the logic is broken
////                break;
//            case FAR_HARD_SHOT:
//                setHoodPosition(0.45);
//                setTargetRPM(3700);
//                break;
//            case MEDIUM_SHOT:
//                setHoodPosition(0.45);
//                setTargetRPM(3400);
//                break;
//            case CLOSE_SHOT:
//                setHoodPosition(0.65);
//                setTargetRPM(3300);
//                break;
//            case NO_SHOT:
//                setHoodPosition(0.30);
//                break;
//        }
//    }
//
//
//    private final double turretLenience = 5; // degrees
//
//    public boolean IsAtCorrectTurretPos()
//    {
//        return Math.abs(targetTurretAngle - getTurretAngle()) < turretLenience;
//    }
//
//    public void setHoodPosition(double value) {
//        leftHood.setPosition(value);
//        rightHood.setPosition(value);
//    }
//
//    // Public getters for telemetry
//    public double getTargetRPM() { return targetRPM; }
//
//    public double getDistance() { return limeLight.GetDistance();}
//    public double getCurrentRPM() {
//        return (rightShooter.getVelocity() / COUNTS_PER_WHEEL_REV) * 60.0;
//    }
//    public double getTurretAngle() {
//        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
//    }
//    public int getTurretTicks() { return turretMotor.getCurrentPosition(); }
//    public TurretState getTurretState() { return currentState; }
//    public double getLimelightYaw() { return lastLimelightYaw; }
//    public double getLastTargetAngle() { return lastCalculatedTargetAngle; }
//    public boolean isCalibrationMode() { return calibrationMode; }
//    public double getLastTX() { return lastTX; }
//    public double getDeadband() { return deadband; }
//}
