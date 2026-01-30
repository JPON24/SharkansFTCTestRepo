package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class swerve {

    // Geometry
    private final double L = 0.98; // Half length
    private final double W = 1.00; // Half width

    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private CRServo frontLeftServo, frontRightServo, backLeftServo, backRightServo;
    private AnalogInput frontLeftAnalog, frontRightAnalog, backLeftAnalog, backRightAnalog;
    private SparkFunOTOS otos;

    private PIDController flPID, frPID, rlPID, rrPID;
    private ElapsedTime pidTimer = new ElapsedTime();

    // PID Constants
    private double FLkP = 0.0035, FLkI = 0.0001, FLkD = 0.0001;
    private double FRkP = 0.0035, FRkI = 0.0001, FRkD = 0.0001;
    private double BLkP = 0.0035, BLkI = 0.0001, BLkD = 0.0001;
    private double BRkP = 0.0035, BRkI = 0.0001, BRkD = 0.0001;
    private double minServoPower = 0.03;
    private double ANGLE_HOLD_SPEED = 0.05;

    // Offsets
    private double FL_OFFSET = -84;
    private double FR_OFFSET = 138;
    private double BL_OFFSET = 85;
    private double BR_OFFSET = -125;

    private double globalSpeedScale = 0.80;

    // State variables
    private double lastTargetFL = 0, lastTargetFR = 0, lastTargetRL = 0, lastTargetRR = 0;

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, null);
    }

    public void init(HardwareMap hardwareMap, SparkFunOTOS otosRef) {
        this.otos = otosRef;

        // Motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Servos
        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        // Encoders (Analog)
        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");
        frontRightAnalog = hardwareMap.get(AnalogInput.class, "frontRightAnalog");
        backLeftAnalog = hardwareMap.get(AnalogInput.class, "backLeftAnalog");
        backRightAnalog = hardwareMap.get(AnalogInput.class, "backRightAnalog");

        // Directions (KEPT YOUR ORIGINAL CONFIG)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize PIDs
        flPID = new PIDController(FLkP, FLkI, FLkD);
        frPID = new PIDController(FRkP, FRkI, FRkD);
        rlPID = new PIDController(BLkP, BLkI, BLkD);
        rrPID = new PIDController(BRkP, BRkI, BRkD);

        // Initialize targets to current position so wheels don't snap on start
        lastTargetFL = getAngle(frontLeftAnalog, FL_OFFSET);
        lastTargetFR = getAngle(frontRightAnalog, FR_OFFSET);
        lastTargetRL = getAngle(backLeftAnalog, BL_OFFSET);
        lastTargetRR = getAngle(backRightAnalog, BR_OFFSET);
    }

    public void drive(double y_cmd, double x_cmd, double turn_cmd) {
        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stop();
            return;
        }

        // Swerve Kinematics (Field Centric if OTOS used, Robot Centric otherwise)
//        if (otos != null) {
//            double heading = otos.getPosition().h;
//            double cos = Math.cos(-heading);
//            double sin = Math.sin(-heading);
//            double temp = x_cmd * cos - y_cmd * sin;
//            y_cmd = x_cmd * sin + y_cmd * cos;
//            x_cmd = temp;
//        }

        // Wheel Vectors
        double y_fr = y_cmd + turn_cmd * L;
        double x_fr = x_cmd - turn_cmd * W;

        double y_fl = y_cmd - turn_cmd * L;
        double x_fl = x_cmd - turn_cmd * W;

        double y_rl = y_cmd - turn_cmd * L;
        double x_rl = x_cmd + turn_cmd * W;

        double y_rr = y_cmd + turn_cmd * L;
        double x_rr = x_cmd + turn_cmd * W;

        // Calculate Speeds
        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        // Calculate Angles
        // If speed is too low, keep the last known angle to prevent jitter
        double angleFL = (speed_fl < ANGLE_HOLD_SPEED) ? lastTargetFL : Math.toDegrees(Math.atan2(x_fl, y_fl));
        double angleFR = (speed_fr < ANGLE_HOLD_SPEED) ? lastTargetFR : Math.toDegrees(Math.atan2(x_fr, y_fr));
        double angleRL = (speed_rl < ANGLE_HOLD_SPEED) ? lastTargetRL : Math.toDegrees(Math.atan2(x_rl, y_rl));
        double angleRR = (speed_rr < ANGLE_HOLD_SPEED) ? lastTargetRR : Math.toDegrees(Math.atan2(x_rr, y_rr));

        // Normalize Speeds (if any > 1.0)
        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
        if (max > 1.0) {
            speed_fr /= max;
            speed_fl /= max;
            speed_rl /= max;
            speed_rr /= max;
        }

        // --- NO OPTIMIZATION HERE ---
        // Just set the power directly based on the kinematics

        frontLeftMotor.setPower(speed_fl * globalSpeedScale);
        frontRightMotor.setPower(speed_fr * globalSpeedScale);
        backLeftMotor.setPower(speed_rl * globalSpeedScale);
        backRightMotor.setPower(speed_rr * globalSpeedScale);

        // Update last targets
        lastTargetFL = angleFL;
        lastTargetFR = angleFR;
        lastTargetRL = angleRL;
        lastTargetRR = angleRR;

        // Run PIDs to turn servos to targets
        runPID(angleFL, angleFR, angleRL, angleRR);
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftServo.setPower(0);
        frontRightServo.setPower(0);
        backLeftServo.setPower(0);
        backRightServo.setPower(0);
    }

    private void runPID(double targetFL, double targetFR, double targetRL, double targetRR) {
        double currentFL = getAngle(frontLeftAnalog, FL_OFFSET);
        double currentFR = getAngle(frontRightAnalog, FR_OFFSET);
        double currentRL = getAngle(backLeftAnalog, BL_OFFSET);
        double currentRR = getAngle(backRightAnalog, BR_OFFSET);

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

    private double getAngle(AnalogInput sensor, double offset) {
        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;
        double adjustedAngle = rawAngle - offset;
        return normalizeAngle(adjustedAngle);
    }

    private double normalizeAngle(double angle) {
        // Normalizes to -180 to 180
        angle = (angle + 180.0) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    // Telemetry
    public double getFLAngle() { return getAngle(frontLeftAnalog, FL_OFFSET); }
    public double getFRAngle() { return getAngle(frontRightAnalog, FR_OFFSET); }
    public double getBLAngle() { return getAngle(backLeftAnalog, BL_OFFSET); }
    public double getBRAngle() { return getAngle(backRightAnalog, BR_OFFSET); }

    public class PIDController {
        private double kP, kI, kD;
        private double lastError = 0;
        private double integralSum = 0;
        private double maxIntegral = 0.5;

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double calculate(double target, double current, double dt) {
            double error = normalizeAngle(target - current);

            double pTerm = error * kP;

            integralSum += error * dt;
            integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
            double iTerm = integralSum * kI;

            double derivative = (error - lastError) / dt;
            double dTerm = derivative * kD;

            lastError = error;

            double output = pTerm + iTerm + dTerm;

            if (Math.abs(error) > 1.25) {
                output += Math.signum(output) * minServoPower;
            } else {
                output = 0;
                integralSum = 0;
            }

            return Range.clip(output, -1.0, 1.0);
        }
    }
}
