package com.sharklib.core.drivetrain;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.sharklib.core.hardware.Motor;
import com.sharklib.core.hardware.SmartCRPServo;
import com.sharklib.core.util.math.geometry.RobotGeometry;

public class CoaxialSwerve {

    RobotGeometry dimensions;

    SmartCRPServo frontLeftServo;
    SmartCRPServo frontRightServo;
    SmartCRPServo backLeftServo;
    SmartCRPServo backRightServo;

    Motor frontLeftMotor;
    Motor frontRightMotor;
    Motor backLeftMotor;
    Motor backRightMotor;

    double minServoPower = 0.03;

    double ANGLE_HOLD_SPEED = 0.05;

    double FL_OFFSET = 0.0;
    double FR_OFFSET = 0.0;
    double BL_OFFSET = 0.0;
    double BR_OFFSET = 0.0;

    double speed = 1.00;

    double lastTargetFL = 0, lastTargetFR = 0, lastTargetRL = 0, lastTargetRR = 0;

    double flSpeed, frSpeed, blSpeed, brSpeed;
    double angleFL, angleFR, angleRL, angleRR;

    /**
     * In op Mode it should look like this:
     *
     * swerve.initFrontLeft(hardwareMap, "flServo", "flMotor", "flEnc", 0.015,0,0.0003,0, 12.5);
     * swerve.initFrontRight(hardwareMap, "frServo", "frMotor", "frEnc", 0.015,0,0.0003,0, 45);
     * swerve.initBackLeft(hardwareMap, "blServo", "blMotor", "blEnc", 0.015,0,0.0003,0, 90);
     * swerve.initBackRight(hardwareMap, "brServo", "brMotor", "brEnc", 0.015,0,0.0003,0, 180);
     *
     * swerve.CoaxialSwerveConstants(11, 10, 9.11) Should be in a ratio.
     */

    public void CoaxialSwerveConstants(double width, double length, double height, double speed)
    {

        this.dimensions = new RobotGeometry(width, length, height);
        this.speed = speed;
    }

    // Front Left
    public void initFrontLeft(HardwareMap hwMap,
                              String servoName,
                              String motorName,
                              String encoderName,
                              double kP, double kI, double kD, double kF,
                              double offset)
    {
        frontLeftServo = new SmartCRPServo(hwMap, servoName, encoderName, kP, kI, kD, kF, 0.3);
        FL_OFFSET = offset;
        frontLeftMotor = new Motor(hwMap, motorName);
    }

    // Front Right
    public void initFrontRight(HardwareMap hwMap,
                               String servoName,
                               String motorName,
                               String encoderName,
                               double kP, double kI, double kD, double kF,
                               double offset)
    {
        frontRightServo = new SmartCRPServo(hwMap, servoName, encoderName, kP, kI, kD, kF, 0.3);
        FR_OFFSET = offset;
        frontRightMotor = new Motor(hwMap, motorName);
    }

    // Back Left
    public void initBackLeft(HardwareMap hwMap,
                             String servoName,
                             String motorName,
                             String encoderName,
                             double kP, double kI, double kD, double kF,
                             double offset)
    {
        backLeftServo = new SmartCRPServo(hwMap, servoName, encoderName, kP, kI, kD, kF, 0.3);
        BL_OFFSET = offset;
        backLeftMotor = new Motor(hwMap, motorName);
    }

    // Back Right
    public void initBackRight(HardwareMap hwMap,
                              String servoName,
                              String motorName,
                              String encoderName,
                              double kP, double kI, double kD, double kF,
                              double offset)
    {
        backRightServo = new SmartCRPServo(hwMap, servoName, encoderName, kP, kI, kD, kF,0.3);
        BR_OFFSET = offset;
        backRightMotor = new Motor(hwMap, motorName);
    }


    public void drive(double x_cmd, double y_cmd, double turn_cmd, double heading) {

        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stop();
            return;
        }

        double rotX = -x_cmd * Math.cos(heading) + y_cmd * Math.sin(heading);
        double rotY = -x_cmd * Math.sin(heading) - y_cmd * Math.cos(heading);

        x_cmd = rotX;
        y_cmd = rotY;

        double y_fr = y_cmd + turn_cmd * dimensions.length;
        double x_fr = x_cmd + turn_cmd * dimensions.width;

        double y_fl = y_cmd - turn_cmd * dimensions.length;
        double x_fl = x_cmd + turn_cmd * dimensions.width;

        double y_rl = y_cmd - turn_cmd * dimensions.length;;
        double x_rl = x_cmd - turn_cmd * dimensions.width;

        double y_rr = y_cmd + turn_cmd * dimensions.length;;
        double x_rr = x_cmd - turn_cmd * dimensions.width;

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
        double currentFL = frontLeftServo.getAngle();
        double currentFR = frontRightServo.getAngle();
        double currentRL = backLeftServo.getAngle();
        double currentRR = backRightServo.getAngle();

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

        runPID(optFL[0], optFR[0], optRL[0], optRR[0]);
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftServo.setPosition(lastTargetFL);
        frontRightServo.setPosition(lastTargetFR);
        backLeftServo.setPosition(lastTargetRL);
        backRightServo.setPosition(lastTargetRR);
    }

    private void runPID(double targetFL, double targetFR, double targetRL, double targetRR) {

        frontLeftServo.setPosition(targetFL);
        frontRightServo.setPosition(targetFR);
        backLeftServo.setPosition(targetRL);
        backRightServo.setPosition(targetRR);
    }

    private void stopDropRoll() {
        drive(0, 0, 0.01, 0);
        drive(0, 0, 0.01, 0);
        drive(0, 0, 0.01, 0);
        drive(0, 0, 0.01, 0);
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
}