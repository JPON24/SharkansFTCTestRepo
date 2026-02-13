package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class WorkingSwerve {

    // Robot dimensions lowkey in a ratio...
    final double L = 0.98;
    final double W = 1.00;

    // Hardware
    DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    CRServo frontLeftServo, frontRightServo, backLeftServo, backRightServo;
    AnalogInput frontLeftAnalog, frontRightAnalog, backLeftAnalog, backRightAnalog;

    // PID Controllers for each module
    PIDController flPID, frPID, rlPID, rrPID;
    ElapsedTime pidTimer = new ElapsedTime();

    SparkFunOTOS otos;

    // PID Constants
    double FLkP = 0.0023;
    double FLkI = 0.0008;
    double FLkD = 0.00008;

    double FRkP = 0.0024;
    double FRkI = 0.0005;
    double FRkD = 0.00008;

    double BLkP = 0.0022;
    double BLkI = 0.0007;
    double BLkD = 0.00008;

    double BRkP = 0.0024;
    double BRkI = 0.0005;
    double BRkD = 0.00008;

    double ANGLE_HOLD_SPEED = 0.05;


    double FL_OFFSET = -179.0;
    double FR_OFFSET = 170; //170
    double BL_OFFSET = -179.0;
    double BR_OFFSET = 166; //166

    double speed = 0.75;

    double lastTargetFL = 0, lastTargetFR = 0, lastTargetRL = 0, lastTargetRR = 0;

    double flSpeed, frSpeed, blSpeed, brSpeed;
    double angleFL, angleFR, angleRL, angleRR;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");
        frontRightAnalog = hardwareMap.get(AnalogInput.class, "frontRightAnalog");
        backLeftAnalog = hardwareMap.get(AnalogInput.class, "backLeftAnalog");
        backRightAnalog = hardwareMap.get(AnalogInput.class, "backRightAnalog");

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(new SparkFunOTOS.Pose2D(0,-3.74016,0));
        otos.calibrateImu();

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        flPID = new PIDController(FLkP, FLkI, FLkD);
        frPID = new PIDController(FRkP, FRkI, FRkD);
        rlPID = new PIDController(BLkP, BLkI, BLkD);
        rrPID = new PIDController(BRkP, BRkI, BRkD);

        lastTargetFL = getAngle(frontLeftAnalog, FL_OFFSET);
        lastTargetFR = getAngle(frontRightAnalog, FR_OFFSET);
        lastTargetRL = getAngle(backLeftAnalog, BL_OFFSET);
        lastTargetRR = getAngle(backRightAnalog, BR_OFFSET);
    }

    public void swerveDrive(double y_cmd, double x_cmd, double turn_cmd) {

        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stopDrive();
            return;
        }

        double currentHeading = -otos.getPosition().h;

        double botHeading = Math.toRadians(currentHeading);

        // Rotation buh
        double rotX = -x_cmd * Math.cos(botHeading) + y_cmd * Math.sin(botHeading);
        double rotY = -x_cmd * Math.sin(botHeading) - y_cmd * Math.cos(botHeading);

        x_cmd = rotX;
        y_cmd = rotY;

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

        // Get da currnt wheel angles
        double currentFL = getAngle(frontLeftAnalog, FL_OFFSET);
        double currentFR = getAngle(frontRightAnalog, FR_OFFSET);
        double currentRL = getAngle(backLeftAnalog, BL_OFFSET);
        double currentRR = getAngle(backRightAnalog, BR_OFFSET);

        // Run through optimize
        double[] optFL = optimize(angleFL, speed_fl, currentFL);
        double[] optFR = optimize(angleFR, speed_fr, currentFR);
        double[] optRL = optimize(angleRL, speed_rl, currentRL);
        double[] optRR = optimize(angleRR, speed_rr, currentRR);

        // Set motor speeds... LOWKIRKENUINLY... If it's backwards... JUST REVERSE THE OUTPUT!!
        flSpeed = optFL[1] * speed;
        frSpeed = optFR[1] * speed;
        blSpeed = optRL[1] * speed;
        brSpeed = optRR[1] * speed;

        frontLeftMotor.setPower(flSpeed);
        frontRightMotor.setPower(frSpeed);
        backLeftMotor.setPower(blSpeed);
        backRightMotor.setPower(brSpeed);

        // Run da PID
        lastTargetFL = optFL[0];
        lastTargetFR = optFR[0];
        lastTargetRL = optRL[0];
        lastTargetRR = optRR[0];

        runPID(optFL[0], optFR[0], optRL[0], optRR[0],
                currentFL, currentFR, currentRL, currentRR);
    }

    public void resetOTOSTracking()
    {
        otos.resetTracking();
    }

    private void stopDrive() {
        // Stop all motors
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // Stop all servos
        frontLeftServo.setPower(0);
        frontRightServo.setPower(0);
        backLeftServo.setPower(0);
        backRightServo.setPower(0);

        flSpeed = frSpeed = blSpeed = brSpeed = 0;
    }

    private void runPID(double targetFL, double targetFR, double targetRL, double targetRR,
                        double currentFL, double currentFR, double currentRL, double currentRR) {
        double dt = pidTimer.seconds();
        if (dt < 0.001) dt = 0.001;

        double powerFL = flPID.calculate(targetFL, currentFL, dt);
        double powerFR = frPID.calculate(targetFR, currentFR, dt);
        double powerRL = rlPID.calculate(targetRL, currentRL, dt);
        double powerRR = rrPID.calculate(targetRR, currentRR, dt);

        frontLeftServo.setPower(powerFL);
        frontRightServo.setPower(powerFR);
        backLeftServo.setPower(powerRL);
        backRightServo.setPower(powerRR);

        pidTimer.reset();
    }

    /**
     * absolute encoder gng
     */
    private double getAngle(AnalogInput sensor, double offset) {
        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;
        double adjustedAngle = rawAngle - offset;
        return normalizeAngle(adjustedAngle);
    }

    /**
     * Regular poo poo normalized angle
     */
    private double normalizeAngle(double angle) {
        angle = (angle + 180.0) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    /**
     * Fack You..
     */
    private double[] optimize(double target, double speed, double current) {
        double delta = normalizeAngle(target - current);

        if (Math.abs(delta) > 90) {
            target = normalizeAngle(target + 180);
            speed *= -1;
        }

        return new double[]{target, speed};
    }

    /**
     * Shrimple PID controller
     */
    public class PIDController {
        private double kP, kI, kD;
        private double lastError = 0;
        private double integralSum = 0.0;
        private double maxIntegral = 48;
        private double minServoPower = 0.03;


        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double calculate(double target, double current, double dt) {
            double error = normalizeAngle(target - current);

            double pTerm = error * kP;
            double derivative = (error - lastError) / dt;
            double dTerm = derivative * kD;

            if (dt > 0) {
                integralSum += error * dt;
                integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
            }

            if (error * lastError < 0)
            {
                integralSum = 0;
            }

            double iTerm = integralSum * kI;

            lastError = error;

            double output = pTerm + iTerm + dTerm;

            if (Math.abs(error) > 0.85) {
                output += Math.signum(output) * minServoPower;
            }

            return Range.clip(output, -1.0, 1.0);
        }
    }

    public double getFLRaw() {
        double raw = (frontLeftAnalog.getVoltage() / 3.3) * 360.0 ;
        return normalizeAngle(raw);
    }

    public double getFRRaw() {
        double raw = (frontRightAnalog.getVoltage() / 3.3) * 360.0;
        return normalizeAngle(raw);
    }

    public double getBLRaw() {
        double raw = (backLeftAnalog.getVoltage() / 3.3) * 360.0;
        return normalizeAngle(raw);
    }

    public double getBRRaw() {
        double raw = (backRightAnalog.getVoltage() / 3.3) * 360.0;
        return normalizeAngle(raw);
    }


    public double getFLError() {
        return lastTargetFL - getAngle(frontLeftAnalog, FL_OFFSET);
    }

    public double getFRError() {
        return lastTargetFR - getAngle(frontRightAnalog, FR_OFFSET);
    }

    public double getBLError() {
        return lastTargetRL - getAngle(backLeftAnalog, BL_OFFSET);
    }

    public double getBRError() {
        return lastTargetRR - getAngle(backRightAnalog, BR_OFFSET);
    }

    public double getMaxError() {
        return Math.max(
                Math.max(Math.abs(getFLError()), Math.abs(getFRError())),
                Math.max(Math.abs(getBLError()), Math.abs(getBRError()))
        );
    }
}
