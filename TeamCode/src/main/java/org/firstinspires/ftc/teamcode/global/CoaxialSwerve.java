package org.firstinspires.ftc.teamcode.global;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class CoaxialSwerve {

    RobotGeometry dimensions;

    public SmartCRPServo frontLeftServo;
    public SmartCRPServo frontRightServo;
    public SmartCRPServo backLeftServo;
    public SmartCRPServo backRightServo;

    Motor frontLeftMotor;
    Motor frontRightMotor;
    Motor backLeftMotor;
    Motor backRightMotor;

    double minServoPower = 0.03;

    double ANGLE_HOLD_SPEED = 0.05;

    double speed = 0;

    double lastTargetFL = 0, lastTargetFR = 0, lastTargetRL = 0, lastTargetRR = 0;
    boolean initialized = false;

    double flSpeed, frSpeed, blSpeed, brSpeed;
    double angleFL, angleFR, angleRL, angleRR;

    /**
     * In op Mode it should look like this:
     *
     * // In init():
     * swerve.initFrontLeft(hardwareMap, "flServo", "flMotor", "flEnc", 0.015,0,0.0003,0, 12.5);
     * swerve.initFrontRight(hardwareMap, "frServo", "frMotor", "frEnc", 0.015,0,0.0003,0, 45);
     * swerve.initBackLeft(hardwareMap, "blServo", "blMotor", "blEnc", 0.015,0,0.0003,0, 90);
     * swerve.initBackRight(hardwareMap, "brServo", "brMotor", "brEnc", 0.015,0,0.0003,0, 180);
     * swerve.CoaxialSwerveConstants(11, 10, 9.11, 0.8);
     * swerve.initialize();  // IMPORTANT!
     *
     * // In loop():
     * swerve.drive(x, y, turn, heading);  // This calls updateServos() internally
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
        frontLeftServo = new SmartCRPServo(hwMap, servoName, encoderName, kP, kI, kD, kF, 0, offset);
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
        frontRightServo = new SmartCRPServo(hwMap, servoName, encoderName, kP, kI, kD, kF, 0, offset);
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
        backLeftServo = new SmartCRPServo(hwMap, servoName, encoderName, kP, kI, kD, kF, 0, offset);
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
        backRightServo = new SmartCRPServo(hwMap, servoName, encoderName, kP, kI, kD, kF,0, offset);
        backRightMotor = new Motor(hwMap, motorName);
    }

    /**
     * Call this after all init methods to set initial positions
     * This prevents servos from spinning to 0 on startup
     */
    public void initialize() {
        // Read current angles from encoders
        lastTargetFL = frontLeftServo.getAngle();
        lastTargetFR = frontRightServo.getAngle();
        lastTargetRL = backLeftServo.getAngle();
        lastTargetRR = backRightServo.getAngle();

        // Set servos to hold current positions
        frontLeftServo.setPosition(lastTargetFL);
        frontRightServo.setPosition(lastTargetFR);
        backLeftServo.setPosition(lastTargetRL);
        backRightServo.setPosition(lastTargetRR);

        initialized = true;
    }

    /**
     * MUST be called every loop iteration to update servo PID controllers
     * This is what actually moves the servos to their targets
     */
    public void updateServos() {
        if (!initialized) {
            return;
        }

        // Update each servo's PID controller
        // This reads encoders, calculates PID, and applies power
        frontLeftServo.update();
        frontRightServo.update();
        backLeftServo.update();
        backRightServo.update();
    }

    public void drive(double x_cmd, double y_cmd, double turn_cmd, double heading) {

        if (!initialized) {
            initialize();
        }

        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stop();
            updateServos(); // Still update PID when stopped to hold position
            return;
        }

        double currentHeading = heading;
        double headingOffset = 0.0;

        double botHeading = Math.toRadians(currentHeading - headingOffset);

        // Rotation buh
        double rotX = -x_cmd * Math.cos(botHeading) + y_cmd * Math.sin(botHeading);
        double rotY = -x_cmd * Math.sin(botHeading) - y_cmd * Math.cos(botHeading);

        x_cmd = rotX;
        y_cmd = rotY;

        double y_fr = y_cmd + turn_cmd * dimensions.length;
        double x_fr = x_cmd + turn_cmd * dimensions.width;

        double y_fl = y_cmd - turn_cmd * dimensions.length;
        double x_fl = x_cmd + turn_cmd * dimensions.width;

        double y_rl = y_cmd - turn_cmd * dimensions.length;
        double x_rl = x_cmd - turn_cmd * dimensions.width;

        double y_rr = y_cmd + turn_cmd * dimensions.length;
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

        // Set motor speeds
        flSpeed = optFL[1] * speed;
        frSpeed = optFR[1] * speed;
        blSpeed = optRL[1] * speed;
        brSpeed = optRR[1] * speed;

        frontLeftMotor.setPower(flSpeed);
        frontRightMotor.setPower(frSpeed);
        backLeftMotor.setPower(blSpeed);
        backRightMotor.setPower(brSpeed);

        // Store last targets
        lastTargetFL = optFL[0];
        lastTargetFR = optFR[0];
        lastTargetRL = optRL[0];
        lastTargetRR = optRR[0];

        // Set servo targets (just stores the target, doesn't move yet)
        setTargets(optFL[0], optFR[0], optRL[0], optRR[0]);

        // Actually move the servos by updating PID controllers
        updateServos();
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // Keep servos at last target position
        setTargets(lastTargetFL, lastTargetFR, lastTargetRL, lastTargetRR);
    }

    private void setTargets(double targetFL, double targetFR, double targetRL, double targetRR) {
        frontLeftServo.setPosition(targetFL);
        frontRightServo.setPosition(targetFR);
        backLeftServo.setPosition(targetRL);
        backRightServo.setPosition(targetRR);
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
     * Public Getters
     * @return returns the voltage of an AXON servo
     */

    public double getFLVoltage() {
        return this.frontLeftServo.getVoltage();
    }

    public double getFRVoltage() {
        return this.frontRightServo.getVoltage();
    }

    public double getBLVoltage() {
        return this.backLeftServo.getVoltage();
    }

    public double getBRVoltage() {
        return this.backRightServo.getVoltage();
    }

    // Diagnostic methods
    public double getFLAngle() {
        return this.frontLeftServo.getAngle();
    }

    public double getFRAngle() {
        return this.frontRightServo.getAngle();
    }

    public double getBLAngle() {
        return this.backLeftServo.getAngle();
    }

    public double getBRAngle() {
        return this.backRightServo.getAngle();
    }
}