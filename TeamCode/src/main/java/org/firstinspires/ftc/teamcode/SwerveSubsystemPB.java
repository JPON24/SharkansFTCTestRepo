package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class SwerveSubsystem {

    private final double L = 0.98;
    private final double W = 1.00;

    // ===== DEADSPOT KEEP-OUT ZONE CONFIGURATION =====
    // Mechanical deadspot is approximately 315° through 0° (wraps around)
    // We define a forbidden zone with hysteresis to prevent commanding angles in this region
    private final double DEADSPOT_START = 315.0;      // Start of forbidden zone (degrees)
    private final double DEADSPOT_END_ENTER = 20.0;   // End of forbidden zone when entering (degrees, wider)
    private final double DEADSPOT_END_EXIT = 15.0;    // End of forbidden zone when exiting (degrees, narrower)
    private final double DEADSPOT_BOUNDARY_MARGIN = 5.0; // Safety margin from boundaries (degrees)

    // Track which modules are currently in "avoiding deadspot" state for hysteresis
    private boolean flInDeadspotAvoidance = false;
    private boolean frInDeadspotAvoidance = false;
    private boolean blInDeadspotAvoidance = false;
    private boolean brInDeadspotAvoidance = false;

    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private Servo frontLeftServo, frontRightServo, backLeftServo, backRightServo;
    private AnalogInput frontLeftAnalog, frontRightAnalog, backLeftAnalog, backRightAnalog;
    private SparkFunOTOS otos;

    private PIDController flPID, frPID, rlPID, rrPID;

    private AxonAnalogFilter flFilter = new AxonAnalogFilter(0.25);
    private AxonAnalogFilter frFilter = new AxonAnalogFilter(0.25);
    private AxonAnalogFilter blFilter = new AxonAnalogFilter(0.25);
    private AxonAnalogFilter brFilter = new AxonAnalogFilter(0.25);

    private ElapsedTime pidTimer = new ElapsedTime();

    // 0.004 0.0001 0.0003
    // 1.68 0.98 0.08
    // 0.168 0.119 0.007
    private double FLkP = 0.004, FLkI = 0.0001, FLkD = 0.0003;
    private double FRkP = 0.004, FRkI = 0.0001, FRkD = 0.0003;
    private double BLkP = 0.004, BLkI = 0.0001, BLkD = 0.0003; // 0.0002
    private double BRkP = 0.004, BRkI = 0.0001, BRkD = 0.0003;
    private double minServoPower = 0.03;
    private double ANGLE_HOLD_SPEED = 0.05;

    private double FL_OFFSET = 0.5;
    private double FR_OFFSET = 0.5;
    private double BL_OFFSET = 0.5;
    private double BR_OFFSET = 0.5;

    double lastWriteFL = 0.0;
    double lastWriteFR = 0.0;
    double lastWriteBL = 0.0;
    double lastWriteBR = 0.0;

    double SERVO_JITTER_THRESHOLD = 0.00396825396; // The math for this is DEGREES OF JITTER / 315. This is 1.25 degrees

    private double headingOffset = 0;

    ElapsedTime updateLimiter = new ElapsedTime();
    private final double swerveUpdateHz = 6;
    private double deltaMax = 25;

    private double speed = 0.85;
    public double lastTargetFL = 0, lastTargetFR = 0, lastTargetRL = 0, lastTargetRR = 0;
    public double flSpeed, frSpeed, blSpeed, brSpeed;
    public double angleFL, angleFR, angleRL, angleRR;

    public void init(HardwareMap hardwareMap, SparkFunOTOS otosRef) {

        this.otos = otosRef;

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(Servo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(Servo.class, "backLeftServo");
        backRightServo = hardwareMap.get(Servo.class, "backRightServo");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flPID = new PIDController(FLkP, FLkI, FLkD);
        frPID = new PIDController(FRkP, FRkI, FRkD);
        rlPID = new PIDController(BLkP, BLkI, BLkD);
        rrPID = new PIDController(BRkP, BRkI, BRkD);

        lastTargetFL = getFLAngle();
        lastTargetFR = getFRAngle();
        lastTargetRL = getBLAngle();
        lastTargetRR = getBRAngle();
    }

    ElapsedTime driveTime = new ElapsedTime();
    private final double decelerationTime = 1;
    public boolean decelerating = false;

    public void resetHeading() {
        if (otos != null) {
            headingOffset = otos.getPosition().h;
        }
    }

    public double heading()
    {
        return otos.getPosition().h;
    }

    public void plant()
    {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public double xCmdVal = 0;
    public double yCmdVal = 0;
    public double rCmdVal = 0;

    public void drive(double y_cmd, double x_cmd, double turn_cmd) {
//        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
//            stop();
//            return;
//        }

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double currentHeading = otos.getPosition().h;

        double botHeading = Math.toRadians(currentHeading - headingOffset);

        // Rotation buh
        double rotX = -x_cmd * Math.cos(botHeading) + y_cmd * Math.sin(botHeading);
        double rotY = -x_cmd * Math.sin(botHeading) - y_cmd * Math.cos(botHeading);

        x_cmd = rotX;
        y_cmd = rotY;

        xCmdVal = x_cmd;
        yCmdVal = y_cmd;
        rCmdVal = turn_cmd;

        double y_fr = y_cmd + turn_cmd * L;
        double x_fr = x_cmd + turn_cmd * W;

        double y_fl = y_cmd - turn_cmd * L;
        double x_fl = x_cmd + turn_cmd * W;

        double y_rl = y_cmd - turn_cmd * L;
        double x_rl = x_cmd - turn_cmd * W;

        double y_rr = y_cmd + turn_cmd * L;
        double x_rr = x_cmd - turn_cmd * W;

        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        angleFL = (speed_fl < ANGLE_HOLD_SPEED) ? lastTargetFL
                : Math.toDegrees(Math.atan2(x_fl, y_fl));
        angleFR = (speed_fr < ANGLE_HOLD_SPEED) ? lastTargetFR
                : Math.toDegrees(Math.atan2(x_fr, y_fr));
        angleRL = (speed_rl < ANGLE_HOLD_SPEED) ? lastTargetRL
                : Math.toDegrees(Math.atan2(x_rl, y_rl));
        angleRR = (speed_rr < ANGLE_HOLD_SPEED) ? lastTargetRR
                : Math.toDegrees(Math.atan2(x_rr, y_rr));

        // previously was subtracting current heading here
        angleFL = Clamp360(angleFL - 22.5) - (FL_OFFSET * 315);
        angleFR = Clamp360(angleFR - 22.5) - (FR_OFFSET * 315);
        angleRL = Clamp360(angleRL - 22.5) - (BL_OFFSET * 315);
        angleRR = Clamp360(angleRR - 22.5) - (BR_OFFSET * 315);

        angleFL = Clamp360(angleFL);
        angleFR = Clamp360(angleFR);
        angleRL = Clamp360(angleRL);
        angleRR = Clamp360(angleRR);

        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
        if (max > 1.0) {
            speed_fr /= max;
            speed_fl /= max;
            speed_rl /= max;
            speed_rr /= max;
        }

        flSpeed = speed_fl * speed;
        frSpeed = speed_fr * speed;
        blSpeed = speed_rl * speed;
        brSpeed = speed_rr * speed;

        double[] optFL = Clamp315(angleFL, flSpeed);
        double[] optFR = Clamp315(angleFR, frSpeed);
        double[] optBL = Clamp315(angleRL, blSpeed);
        double[] optBR = Clamp315(angleRR, brSpeed);

        double[] optParamsFL = optimizeWithDeadspot(optFL[0], optFL[1], lastTargetFL, flInDeadspotAvoidance);
        double[] optParamsFR = optimizeWithDeadspot(optFR[0], optFR[1], lastTargetFR, frInDeadspotAvoidance);
        double[] optParamsRL = optimizeWithDeadspot(optBL[0], optBL[1], lastTargetRL, blInDeadspotAvoidance);
        double[] optParamsRR = optimizeWithDeadspot(optBR[0], optBR[1], lastTargetRR, brInDeadspotAvoidance);

        // Update hysteresis state for each module
        flInDeadspotAvoidance = isInDeadspot(optParamsFL[0], DEADSPOT_END_EXIT);
        frInDeadspotAvoidance = isInDeadspot(optParamsFR[0], DEADSPOT_END_EXIT);
        blInDeadspotAvoidance = isInDeadspot(optParamsRL[0], DEADSPOT_END_EXIT);
        brInDeadspotAvoidance = isInDeadspot(optParamsRR[0], DEADSPOT_END_EXIT);

        double tgtPosFL = GetPositionFromAngle(optParamsFL[0], FL_OFFSET);
        double tgtPosFR = GetPositionFromAngle(optParamsFR[0], FR_OFFSET);
        double tgtPosRL = GetPositionFromAngle(optParamsRL[0], BL_OFFSET);
        double tgtPosRR = GetPositionFromAngle(optParamsRR[0], BR_OFFSET);

        optFL = CorrectOutOfRange(tgtPosFL, optParamsFL[1], (BR_OFFSET-FL_OFFSET));
        optFR = CorrectOutOfRange(tgtPosFR, optParamsFR[1], (BR_OFFSET-FR_OFFSET));
        optBL = CorrectOutOfRange(tgtPosRL, optParamsRL[1], (BR_OFFSET-BL_OFFSET));
        optBR = CorrectOutOfRange(tgtPosRR, optParamsRR[1], 0);

        double outputSpeed = 1.0;
//        if (x_cmd == 0 && y_cmd == 0 && turn_cmd == 0 && !decelerating)
//        {
//            decelerating = true;
//            driveTime.reset();
//        }
//        else if (decelerating)
//        {
//            if (driveTime.seconds() > decelerationTime)
//            {
//                decelerating = false;
//                outputSpeed = 0;
//            }
//            else
//            {
//                outputSpeed *= (decelerationTime - driveTime.seconds()) / decelerationTime;
//            }
//        }
        frontLeftMotor.setPower(optFL[1] * outputSpeed);
        frontRightMotor.setPower(optFR[1] * outputSpeed);
        backLeftMotor.setPower(optBL[1] * outputSpeed);
        backRightMotor.setPower(optBR[1] * outputSpeed);

        lastTargetFL = optFL[0];
        lastTargetFR = optFR[0];
        lastTargetRL = optBL[0];
        lastTargetRR = optBR[0];

        SetServoPositions(optFL[0], optFR[0], optBL[0], optBR[0]);
    }

    public void robotCentric(double y_cmd, double x_cmd, double turn_cmd) {
//        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
//            stop();
//            return;
//        }

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        xCmdVal = x_cmd;
        yCmdVal = y_cmd;
        rCmdVal = turn_cmd;

        double y_fr = y_cmd + turn_cmd * L;
        double x_fr = x_cmd + turn_cmd * W;

        double y_fl = y_cmd - turn_cmd * L;
        double x_fl = x_cmd + turn_cmd * W;

        double y_rl = y_cmd - turn_cmd * L;
        double x_rl = x_cmd - turn_cmd * W;

        double y_rr = y_cmd + turn_cmd * L;
        double x_rr = x_cmd - turn_cmd * W;

        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        angleFL = (speed_fl < ANGLE_HOLD_SPEED) ? lastTargetFL
                : Math.toDegrees(Math.atan2(x_fl, y_fl));
        angleFR = (speed_fr < ANGLE_HOLD_SPEED) ? lastTargetFR
                : Math.toDegrees(Math.atan2(x_fr, y_fr));
        angleRL = (speed_rl < ANGLE_HOLD_SPEED) ? lastTargetRL
                : Math.toDegrees(Math.atan2(x_rl, y_rl));
        angleRR = (speed_rr < ANGLE_HOLD_SPEED) ? lastTargetRR
                : Math.toDegrees(Math.atan2(x_rr, y_rr));

        // previously was subtracting current heading here
        angleFL = Clamp360(angleFL - 22.5) - (FL_OFFSET * 315);
        angleFR = Clamp360(angleFR - 22.5) - (FR_OFFSET * 315);
        angleRL = Clamp360(angleRL - 22.5) - (BL_OFFSET * 315);
        angleRR = Clamp360(angleRR - 22.5) - (BR_OFFSET * 315);

        angleFL = Clamp360(angleFL);
        angleFR = Clamp360(angleFR);
        angleRL = Clamp360(angleRL);
        angleRR = Clamp360(angleRR);

        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
        if (max > 1.0) {
            speed_fr /= max;
            speed_fl /= max;
            speed_rl /= max;
            speed_rr /= max;
        }

        flSpeed = speed_fl * speed;
        frSpeed = speed_fr * speed;
        blSpeed = speed_rl * speed;
        brSpeed = speed_rr * speed;

        double[] optFL = Clamp315(angleFL, flSpeed);
        double[] optFR = Clamp315(angleFR, frSpeed);
        double[] optBL = Clamp315(angleRL, blSpeed);
        double[] optBR = Clamp315(angleRR, brSpeed);

        double[] optParamsFL = optimizeWithDeadspot(optFL[0], optFL[1], lastTargetFL, flInDeadspotAvoidance);
        double[] optParamsFR = optimizeWithDeadspot(optFR[0], optFR[1], lastTargetFR, frInDeadspotAvoidance);
        double[] optParamsRL = optimizeWithDeadspot(optBL[0], optBL[1], lastTargetRL, blInDeadspotAvoidance);
        double[] optParamsRR = optimizeWithDeadspot(optBR[0], optBR[1], lastTargetRR, brInDeadspotAvoidance);

        // Update hysteresis state for each module
        flInDeadspotAvoidance = isInDeadspot(optParamsFL[0], DEADSPOT_END_EXIT);
        frInDeadspotAvoidance = isInDeadspot(optParamsFR[0], DEADSPOT_END_EXIT);
        blInDeadspotAvoidance = isInDeadspot(optParamsRL[0], DEADSPOT_END_EXIT);
        brInDeadspotAvoidance = isInDeadspot(optParamsRR[0], DEADSPOT_END_EXIT);

        double tgtPosFL = GetPositionFromAngle(optParamsFL[0], FL_OFFSET);
        double tgtPosFR = GetPositionFromAngle(optParamsFR[0], FR_OFFSET);
        double tgtPosRL = GetPositionFromAngle(optParamsRL[0], BL_OFFSET);
        double tgtPosRR = GetPositionFromAngle(optParamsRR[0], BR_OFFSET);

        optFL = CorrectOutOfRange(tgtPosFL, optParamsFL[1], (BR_OFFSET-FL_OFFSET));
        optFR = CorrectOutOfRange(tgtPosFR, optParamsFR[1], (BR_OFFSET-FR_OFFSET));
        optBL = CorrectOutOfRange(tgtPosRL, optParamsRL[1], (BR_OFFSET-BL_OFFSET));
        optBR = CorrectOutOfRange(tgtPosRR, optParamsRR[1], 0);

        double outputSpeed = 1.0;
//        if (x_cmd == 0 && y_cmd == 0 && turn_cmd == 0 && !decelerating)
//        {
//            decelerating = true;
//            driveTime.reset();
//        }
//        else if (decelerating)
//        {
//            if (driveTime.seconds() > decelerationTime)
//            {
//                decelerating = false;
//                outputSpeed = 0;
//            }
//            else
//            {
//                outputSpeed *= (decelerationTime - driveTime.seconds()) / decelerationTime;
//            }
//        }
        frontLeftMotor.setPower(optFL[1] * outputSpeed);
        frontRightMotor.setPower(optFR[1] * outputSpeed);
        backLeftMotor.setPower(optBL[1] * outputSpeed);
        backRightMotor.setPower(optBR[1] * outputSpeed);

        lastTargetFL = optFL[0];
        lastTargetFR = optFR[0];
        lastTargetRL = optBL[0];
        lastTargetRR = optBR[0];

        SetServoPositions(optFL[0], optFR[0], optBL[0], optBR[0]);
    }

    private double Clamp360(double angle)
    {
        if (angle < 0)
        {
            angle += 360;
        }

        if (angle > 360)
        {
            angle -= 360;
        }

        return angle;
    }

    private double[] Clamp315(double angle, double motor)
    {
        double[] output = new double[2];
        if (angle > 315)
        {
            angle -= 225;
            motor *= -1;
        }

        output[0] = angle;
        output[1] = motor;

        return output;
    }

    public void resetIMU()
    {
        otos.resetTracking();
    }


    public double GetAngle(double position, double offset) {
        return (offset - position) * 315;
    }

    public double GetPositionFromAngle(double angle, double offset)
    {
        double position = offset - (angle / 315);

        return position;
    }

    public double[] CorrectOutOfRange(double tgt, double motor, double offset)
    {
        double[] output = new double[2];

        if (tgt < 0)
        {
            tgt += (360.0 / 630);
            motor *= -1;
        }

        output[0] = tgt + offset;
        output[1] = motor;

        return output;
    }

    public void SetServoPositions(double FL, double FR, double BL, double BR) {
        if (updateLimiter.seconds() > 1.0 / swerveUpdateHz)
        {
            if (Math.abs(FL - lastWriteFL) > SERVO_JITTER_THRESHOLD) {
                frontLeftServo.setPosition(FL);
                lastWriteFL = FL;
            }

            if (Math.abs(FR - lastWriteFR) > SERVO_JITTER_THRESHOLD) {
                frontRightServo.setPosition(FR);
                lastWriteFR = FR;
            }

            if (Math.abs(BL - lastWriteBL) > SERVO_JITTER_THRESHOLD) {
                backLeftServo.setPosition(BL);
                lastWriteBL = BL;
            }

            if (Math.abs(BR - lastWriteBR) > SERVO_JITTER_THRESHOLD) {
                backRightServo.setPosition(BR);
                lastWriteBR = BR;
            }
        }
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
//        frontLeftServo.setPosition(0);
//        frontRightServo.setPower(0);
//        backLeftServo.setPower(0);
//        backRightServo.setPower(0);
        flSpeed = frSpeed = blSpeed = brSpeed = 0;
    }

    private final double offsetTimer = 0.6;

    private void runPID(double targetFL, double targetFR, double targetRL, double targetRR,
                        double currentFL, double currentFR, double currentRL, double currentRR) {
        double dt = pidTimer.seconds();
        if (dt < 0.001) dt = 0.001;

        double powerFL = flPID.calculate(targetFL, currentFL, dt);
        double powerFR = frPID.calculate(targetFR, currentFR, dt);

        double powerRL = rlPID.calculate(targetRL, currentRL, dt);
        double powerRR = rrPID.calculate(targetRR, currentRR, dt);

//        frontLeftServo.setPower(powerFL * 1);
//        frontRightServo.setPower(powerFR * 1);
//        backLeftServo.setPower(powerRL * 1);
//        backRightServo.setPower(powerRR * 1);

        pidTimer.reset();
    }

//    private double getAngle(AnalogInput sensor, double offset, AxonAnalogFilter filter) {
//        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;
//        double offsetAngle = normalizeAngle(rawAngle - offset);
//        return filter.estimate(offsetAngle);
//    }

    private double normalizeAngle(double angle) {
        angle = (angle + 180.0) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    /**
     * Check if an angle falls within the forbidden deadspot zone.
     * Deadspot wraps around 360° (e.g., 315° through 20° means [315,360) ∪ [0,20]).
     *
     * @param angle Angle in degrees (0-360)
     * @param endThreshold Use DEADSPOT_END_ENTER or DEADSPOT_END_EXIT for hysteresis
     * @return true if angle is in forbidden zone
     */
    private boolean isInDeadspot(double angle, double endThreshold) {
        angle = Clamp360(angle);

        // Deadspot wraps around 0: check if angle >= start OR angle <= end
        if (angle >= DEADSPOT_START || angle <= endThreshold) {
            return true;
        }
        return false;
    }

    /**
     * Find the nearest allowed angle outside the deadspot.
     * If angle is in deadspot, clamp to the closest valid boundary.
     *
     * @param angle Target angle in degrees
     * @param inAvoidance Current hysteresis state for this module
     * @return Corrected angle outside deadspot
     */
    private double avoidDeadspot(double angle, boolean inAvoidance) {
        angle = Clamp360(angle);

        // Determine which threshold to use (hysteresis)
        double endThreshold = inAvoidance ? DEADSPOT_END_EXIT : DEADSPOT_END_ENTER;

        if (!isInDeadspot(angle, endThreshold)) {
            return angle; // Already safe
        }

        // Angle is in deadspot - find nearest boundary
        // Option 1: Clamp to endThreshold + margin (e.g., 20° or 15°)
        // Option 2: Clamp to DEADSPOT_START - margin (e.g., 310°)

        double distToLowBoundary = angle; // Distance to 0° side
        if (angle > 180) {
            distToLowBoundary = 360 - angle; // Wrap-around distance
        }

        double distToHighBoundary = DEADSPOT_START - angle;
        if (distToHighBoundary < 0) {
            distToHighBoundary += 360;
        }

        // Choose closer boundary and add safety margin
        if (distToLowBoundary < distToHighBoundary) {
            return endThreshold + DEADSPOT_BOUNDARY_MARGIN; // Exit on low side (~20-25°)
        } else {
            return DEADSPOT_START - DEADSPOT_BOUNDARY_MARGIN; // Exit on high side (~310°)
        }
    }

    /**
     * Optimize module steering to minimize rotation while AVOIDING DEADSPOT.
     * Standard swerve optimization: if angle delta > 90°, flip 180° and reverse motor.
     * DEADSPOT-AWARE: Reject any target that falls in forbidden zone, choose valid alternative.
     *
     * @param target Desired steering angle (degrees, 0-360)
     * @param speed Desired drive speed (-1 to 1)
     * @param current Current steering angle (degrees, 0-360)
     * @param inAvoidance Hysteresis state for this module
     * @return [optimized_angle, optimized_speed] - guaranteed outside deadspot
     */
    private double[] optimizeWithDeadspot(double target, double speed, double current, boolean inAvoidance) {
        target = Clamp360(target);
        current = Clamp360(current);

        // Generate two candidate solutions:
        // Candidate A: steer to target, drive at +speed
        // Candidate B: steer to target+180°, drive at -speed
        double candidateA_angle = target;
        double candidateA_speed = speed;

        double candidateB_angle = Clamp360(target + 180);
        double candidateB_speed = -speed;

        // Determine which threshold to use for deadspot check (hysteresis)
        double endThreshold = inAvoidance ? DEADSPOT_END_EXIT : DEADSPOT_END_ENTER;

        // Check if each candidate is valid (outside deadspot)
        boolean aValid = !isInDeadspot(candidateA_angle, endThreshold);
        boolean bValid = !isInDeadspot(candidateB_angle, endThreshold);

        // Calculate rotation needed for each valid candidate
        double deltaA = Math.abs(normalizeAngle(candidateA_angle - current));
        double deltaB = Math.abs(normalizeAngle(candidateB_angle - current));

        // Decision logic:
        // 1. If both valid: choose one requiring less rotation (standard optimization)
        // 2. If only one valid: use that one
        // 3. If neither valid (rare edge case): clamp to nearest boundary

        if (aValid && bValid) {
            // Both valid - pick shorter rotation
            if (deltaA <= deltaB) {
                return new double[]{candidateA_angle, candidateA_speed};
            } else {
                return new double[]{candidateB_angle, candidateB_speed};
            }
        } else if (aValid) {
            // Only A is valid
            return new double[]{candidateA_angle, candidateA_speed};
        } else if (bValid) {
            // Only B is valid
            return new double[]{candidateB_angle, candidateB_speed};
        } else {
            // Neither valid (both in deadspot) - emergency fallback
            // This should be rare - clamp target to nearest safe angle
            double safeAngle = avoidDeadspot(candidateA_angle, inAvoidance);

            // Determine if we should reverse speed based on which candidate was closer
            if (deltaA <= deltaB) {
                return new double[]{safeAngle, candidateA_speed * 0.8}; // Reduce speed slightly near boundary
            } else {
                return new double[]{safeAngle, candidateB_speed * 0.8};
            }
        }
    }

    public void reZero() {
        FL_OFFSET = (frontLeftAnalog.getVoltage() / 3.3) * 360.0;
        FR_OFFSET = (frontRightAnalog.getVoltage() / 3.3) * 360.0;
        BL_OFFSET = (backLeftAnalog.getVoltage() / 3.3) * 360.0;
        BR_OFFSET = (backRightAnalog.getVoltage() / 3.3) * 360.0;
    }

    public void alignWithWall() {

//        frontLeftServo.setPower(0);
//        frontRightServo.setPower(0);
//        backLeftServo.setPower(0);
//        backRightServo.setPower(0);
    }

    public double getFLAngle() { return GetAngle(frontLeftServo.getPosition(), FL_OFFSET);  }
    public double getFRAngle() { return GetAngle(frontRightServo.getPosition(), FR_OFFSET);  }
    public double getBLAngle()
    {
        return GetAngle(backLeftServo.getPosition(), BL_OFFSET);
    }

    public double getBRAngle() { return GetAngle(backRightServo.getPosition(), BR_OFFSET); }

    public double getFLRawAngle() { return (frontLeftAnalog.getVoltage() / 3.3) * 360.0; }
    public double getFRRawAngle() { return (frontRightAnalog.getVoltage() / 3.3) * 360.0; }
    public double getBLRawAngle() { return (backLeftAnalog.getVoltage() / 3.3) * 360.0; }
    public double getBRRawAngle() { return (backRightAnalog.getVoltage() / 3.3) * 360.0; }

    public class PIDController {
        private double kP, kI, kD;
        private double lastError = 0;
        private double integralSum = 0;
        private double maxIntegral = 0;

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double calculate(double target, double current, double dt) {
            target *= 1.0;
            current *= 1.0;
            double error = normalizeAngle(target - current);

            double pTerm = error * kP;

            integralSum += error * dt;
            integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
            double iTerm = integralSum * kI;

            double derivative = (error - lastError) / dt;
            double dTerm = derivative * kD;

            lastError = error;

            if (error * lastError < 0)
            {
                integralSum = 0;
            }

            double output = pTerm + iTerm + dTerm;

            if (Math.abs(error) > 2.00) {
                output += Math.signum(output) * minServoPower;
            } else {
                output = 0;
                integralSum = 0;
            }

            return Range.clip(output, -1.0, 1.0);
        }

        public void resetIntegral() {
            integralSum = 0;
        }
    }
}