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

    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private Servo frontLeftServo, frontRightServo, backLeftServo, backRightServo;
    private AnalogInput frontLeftAnalog, frontRightAnalog, backLeftAnalog, backRightAnalog;
    private SparkFunOTOS otos;

    private PIDController flPID, frPID, rlPID, rrPID;

    private ElapsedTime pidTimer = new ElapsedTime();

    private double FLkP = 0.004, FLkI = 0.0001, FLkD = 0.0003;
    private double FRkP = 0.004, FRkI = 0.0001, FRkD = 0.0003;
    private double BLkP = 0.004, BLkI = 0.0001, BLkD = 0.0003;
    private double BRkP = 0.004, BRkI = 0.0001, BRkD = 0.0003;
    private double minServoPower = 0.03;
    private double ANGLE_HOLD_SPEED = 0.05;

    private double FL_OFFSET = 0.5;
    private double FR_OFFSET = 0.5;
    private double BL_OFFSET = 0.5;
    private double BR_OFFSET = 0.5;

    ElapsedTime updateLimiter = new ElapsedTime();
    private final double swerveUpdateHz = 4;

    private double speed = 0.65;
    public double lastTargetFL = 0, lastTargetFR = 0, lastTargetRL = 0, lastTargetRR = 0;
    private double flSpeed, frSpeed, blSpeed, brSpeed;
    public double angleFL, angleFR, angleRL, angleRR;

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, null);
    }

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

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flPID = new PIDController(FLkP, FLkI, FLkD);
        frPID = new PIDController(FRkP, FRkI, FRkD);
        rlPID = new PIDController(BLkP, BLkI, BLkD);
        rrPID = new PIDController(BRkP, BRkI, BRkD);

        // Initialize sensors if you have them, otherwise these are 0
        // If you don't have encoders connected, these methods might return 0
        lastTargetFL = 0;
        lastTargetFR = 0;
        lastTargetRL = 0;
        lastTargetRR = 0;
    }

    public void drive(double y_cmd, double x_cmd, double turn_cmd) {
        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stop();
            return;
        }

        // Field Centric Code
        if (otos != null) {
            double heading = otos.getPosition().h;
            double cos = Math.cos(Math.toRadians(-heading));
            double sin = Math.sin(Math.toRadians(-heading));
            double temp = x_cmd * cos - y_cmd * sin;
            y_cmd = x_cmd * sin + y_cmd * cos;
            x_cmd = temp;
        }

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

        // 1. Calculate Raw Target Angles
        double rawAngleFL = Math.toDegrees(Math.atan2(x_fl, y_fl));
        double rawAngleFR = Math.toDegrees(Math.atan2(x_fr, y_fr));
        double rawAngleRL = Math.toDegrees(Math.atan2(x_rl, y_rl));
        double rawAngleRR = Math.toDegrees(Math.atan2(x_rr, y_rr));

        // 2. OPTIMIZE HERE
        // This finds the shortest path (-90 to +90) and flips speed if needed.
        // We compare against 'lastTarget' to minimize rotation.
        double[] optParamsFL = optimize(rawAngleFL, speed_fl, lastTargetFL);
        double[] optParamsFR = optimize(rawAngleFR, speed_fr, lastTargetFR);
        double[] optParamsRL = optimize(rawAngleRL, speed_rl, lastTargetRL);
        double[] optParamsRR = optimize(rawAngleRR, speed_rr, lastTargetRR);

        angleFL = (speed_fl < ANGLE_HOLD_SPEED) ? lastTargetFL : optParamsFL[0];
        angleFR = (speed_fr < ANGLE_HOLD_SPEED) ? lastTargetFR : optParamsFR[0];
        angleRL = (speed_rl < ANGLE_HOLD_SPEED) ? lastTargetRL : optParamsRL[0];
        angleRR = (speed_rr < ANGLE_HOLD_SPEED) ? lastTargetRR : optParamsRR[0];

        // 3. Continue with YOUR existing Deadspot/Clamp Logic
        // We feed the optimized angle into your pipeline.
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
            optParamsFL[1] /= max;
            optParamsFR[1] /= max;
            optParamsRL[1] /= max;
            optParamsRR[1] /= max;
        }

        // Use the optimized speeds (which may be negative)
        flSpeed = optParamsFL[1] * speed;
        frSpeed = optParamsFR[1] * speed;
        blSpeed = optParamsRL[1] * speed;
        brSpeed = optParamsRR[1] * speed;

        double[] optFL = Clamp315(angleFL, flSpeed);
        double[] optFR = Clamp315(angleFR, frSpeed);
        double[] optBL = Clamp315(angleRL, blSpeed);
        double[] optBR = Clamp315(angleRR, brSpeed);

        double tgtPosFL = GetPositionFromAngle(optFL[0], FL_OFFSET);
        double tgtPosFR = GetPositionFromAngle(optFR[0], FR_OFFSET);
        double tgtPosRL = GetPositionFromAngle(optBL[0], BL_OFFSET);
        double tgtPosRR = GetPositionFromAngle(optBR[0], BR_OFFSET);

        optFL = CorrectOutOfRange(tgtPosFL, optFL[1], (BR_OFFSET-FL_OFFSET));
        optFR = CorrectOutOfRange(tgtPosFR, optFR[1], (BR_OFFSET-FR_OFFSET));
        optBL = CorrectOutOfRange(tgtPosRL, optBL[1], (BR_OFFSET-BL_OFFSET));
        optBR = CorrectOutOfRange(tgtPosRR, optBR[1], 0);

        frontLeftMotor.setPower(optFL[1]);
        frontRightMotor.setPower(optFR[1]);
        backLeftMotor.setPower(optBL[1]);
        backRightMotor.setPower(optBR[1]);

        lastTargetFL = optParamsFL[0];
        lastTargetFR = optParamsFR[0];
        lastTargetRL = optParamsRL[0];
        lastTargetRR = optParamsRR[0];

        SetServoPositions(optFL[0], optFR[0], optBL[0], optBR[0]);
    }

    // --- NEW HELPER FUNCTIONS ---

    private double[] optimize(double target, double speed, double current) {
        double delta = normalizeAngle(target - current);
        if (Math.abs(delta) > 90) {
            target = normalizeAngle(target + 180);
            speed *= -1;
        }
        return new double[]{target, speed};
    }

    private double normalizeAngle(double angle) {
        angle = (angle + 180.0) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }


    private double Clamp360(double angle) {
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    private double[] Clamp315(double angle, double motor) {
        double[] output = new double[2];
        if (angle > 315) {
            angle -= 225;
            motor *= -1;
        }
        output[0] = angle;
        output[1] = motor;
        return output;
    }

    public double GetAngle(double position, double offset) {
        return (offset - position) * 315;
    }

    public double GetPositionFromAngle(double angle, double offset) {
        double position = offset - (angle / 315);
        return position;
    }

    public double[] CorrectOutOfRange(double tgt, double motor, double offset) {
        double[] output = new double[2];
        if (tgt < 0) {
            tgt += (360.0 / 630);
            motor *= -1;
        }
        output[0] = tgt + offset;
        output[1] = motor;
        return output;
    }

    public void SetServoPositions(double FL, double FR, double BL, double BR) {
        if (updateLimiter.seconds() > 1.0 / swerveUpdateHz) {
            frontLeftServo.setPosition(FL);
            frontRightServo.setPosition(FR);
            backLeftServo.setPosition(BL);
            backRightServo.setPosition(BR);
            updateLimiter.reset();
        }
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        flSpeed = frSpeed = blSpeed = brSpeed = 0;
    }

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
            if (error * lastError < 0) { integralSum = 0; }
            double output = pTerm + iTerm + dTerm;
            if (Math.abs(error) > 2.00) {
                output += Math.signum(output) * minServoPower;
            } else {
                output = 0;
                integralSum = 0;
            }
            return Range.clip(output, -1.0, 1.0);
        }
    }
}