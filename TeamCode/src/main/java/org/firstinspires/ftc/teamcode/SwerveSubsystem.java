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

    private double FL_OFFSET = 0.22;
    private double FR_OFFSET = 0.42;
    private double BL_OFFSET = 0.16;
    private double BR_OFFSET = 0.5;

    ElapsedTime offTimer = new ElapsedTime();
    private double deltaMax = 25;

    private double speed = 0;
    private double lastTargetFL = 0, lastTargetFR = 0, lastTargetRL = 0, lastTargetRR = 0;
    private double flSpeed, frSpeed, blSpeed, brSpeed;
    private double angleFL, angleFR, angleRL, angleRR;

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

//        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");
//        frontRightAnalog = hardwareMap.get(AnalogInput.class, "frontRightAnalog");
//        backLeftAnalog = hardwareMap.get(AnalogInput.class, "backLeftAnalog");
//        backRightAnalog = hardwareMap.get(AnalogInput.class, "backRightAnalog");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flPID = new PIDController(FLkP, FLkI, FLkD);
        frPID = new PIDController(FRkP, FRkI, FRkD);
        rlPID = new PIDController(BLkP, BLkI, BLkD);
        rrPID = new PIDController(BRkP, BRkI, BRkD);

        lastTargetFL = getFLAngle();
        lastTargetFR = getFRAngle();
        lastTargetRL = getBLAngle();
        lastTargetRR = getBRAngle();
    }

    public void drive(double y_cmd, double x_cmd, double turn_cmd) {
        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stop();
            return;
        }

        // Field Centric Code (Uncomment if needed)
        /*
        if (otos != null) {
            double heading = otos.getPosition().h;
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double temp = x_cmd * cos - y_cmd * sin;
            y_cmd = x_cmd * sin + y_cmd * cos;
            x_cmd = temp;
        }
        */

        double y_fr = y_cmd + turn_cmd * L;
        double x_fr = x_cmd - turn_cmd * W;

        double y_fl = y_cmd - turn_cmd * L;
        double x_fl = x_cmd - turn_cmd * W;

        double y_rl = y_cmd - turn_cmd * L;
        double x_rl = x_cmd + turn_cmd * W;

        double y_rr = y_cmd + turn_cmd * L;
        double x_rr = x_cmd + turn_cmd * W;

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

        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
        if (max > 1.0) {
            speed_fr /= max;
            speed_fl /= max;
            speed_rl /= max;
            speed_rr /= max;
        }

        double currentFL = getFLAngle();
        double currentFR = getFRAngle();
        double currentRL = getBLAngle();
        double currentRR = getBRAngle();

        double[] optFL = optimize(angleFL, speed_fl, currentFL);
        double[] optFR = optimize(angleFR, speed_fr, currentFR);
        double[] optRL = optimize(angleRL, speed_rl, currentRL);
        double[] optRR = optimize(angleRR, speed_rr, currentRR);

        flSpeed = optFL[1] * speed;
        frSpeed = optFR[1] * speed;
        blSpeed = optRL[1] * speed;
        brSpeed = optRR[1] * speed;

        frontLeftMotor.setPower(flSpeed);
        frontRightMotor.setPower(frSpeed);
        backLeftMotor.setPower(blSpeed);
        backRightMotor.setPower(brSpeed);

        lastTargetFL = optFL[0];
        lastTargetFR = optFR[0];
        lastTargetRL = optRL[0];
        lastTargetRR = optRR[0];

        SetServoPositions(optFL[0], optFR[0], optRL[0], optRR[0]);

//        runPID(optFL[0], optFR[0], optRL[0], optRR[0],
//                currentFL, currentFR, currentRL, currentRR);
    }

    public void SetServoPositions(double FL, double FR, double BL, double BR)
    {
        frontLeftServo.setPosition(GetPositionFromAngle(FL, FL_OFFSET));
        frontRightServo.setPosition(GetPositionFromAngle(FR, FR_OFFSET));
        backLeftServo.setPosition(GetPositionFromAngle(BL, BL_OFFSET));
        backRightServo.setPosition(GetPositionFromAngle(BR, BR_OFFSET));
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

    public double GetAngle(double position, double offset) {
        return (offset - position) * 315;
    }

    public double GetPositionFromAngle(double angle, double offset)
    {
        double position = offset - (angle / 360);

        if (position < 0)
        {
            position += 1;
        }
        else if (position > 1)
        {
            position -= 1;
        }

        return position;
    }

    private double normalizeAngle(double angle) {
        angle = (angle + 180.0) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    private double[] optimize(double target, double speed, double current) {
        double delta = normalizeAngle(target - current);
        if (Math.abs(delta) > 90) {
            target = normalizeAngle(target + 180);
            speed *= -1;
        }
        return new double[]{target, speed};
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