package org.firstinspires.ftc.teamcode.global;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SmartCRPServo {

    private AnalogFilter filter;

    private CRServo crServo;
    private PIDController pid;
    private AnalogInput encoder;

    private double lastPower = 0.0;
    private double targetAngle = 0.0;  // Store the target angle
    public double THRESHOLD = 0.01;

    public double offset;

    /**
     * Constructor
     * @param hwMap The hardware map
     * @param servoName The name of the CR Servo in config
     * @param encoderName The name of the Analog Encoder in config
     * @param kP The P tuning value
     * @param kI The I tuning value
     * @param kD The D tuning value
     * @param kF The F tuning value
     * @param alpha The alpha for the filter
     * @param offset The encoder offset in degrees
     */
    public SmartCRPServo(HardwareMap hwMap, String servoName, String encoderName, double kP, double kI, double kD, double kF, double alpha, double offset) {
        this.crServo = hwMap.get(CRServo.class, servoName);
        this.encoder = hwMap.get(AnalogInput.class, encoderName);

        this.pid = new PIDController(kP, kI, kD, kF);
        this.filter = new AnalogFilter(alpha);

        this.offset = offset;

        this.targetAngle = getAngle();
    }

    /**
     * Set the target angle - does NOT move the servo yet
     * Call update() repeatedly to actually move to this target
     * @param targetAngle The target angle (e.g., -180 to 180 degrees)
     */
    public void setPosition(double targetAngle) {
        this.targetAngle = LinearMath.angleWrap(targetAngle);
    }

    /**
     * MUST be called repeatedly in your loop (50+ times per second)
     * This actually runs the PID control and moves the servo
     */
    public void update() {
        // Read current angle
        double currentVoltage = encoder.getVoltage();
        double currentAngle = (currentVoltage / 3.3) * 360.0;

        // Apply filter to smooth out noise
        double filterAngle = filter.estimate(currentAngle);
        currentAngle = filterAngle;

        // Apply offset and normalize to -180 to 180
        currentAngle = normalizeAngle(currentAngle - offset);

        // Calculate PID output based on error
        double power = pid.update(targetAngle, currentAngle);

        // Apply the calculated power to the servo
        setPower(power);
    }

    /**
     * Standard manual power control
     */
    public void setPower(double power) {
        // Safety Clamp
        if (power > 1.0) power = 1.0;
        if (power < -1.0) power = -1.0;

        // Caching Logic to reduce CAN bus traffic
        if (Math.abs(power - lastPower) > THRESHOLD || (power == 0 && lastPower != 0)) {
            crServo.setPower(power);
            lastPower = power;
        }
    }

    // Helper to tune PID values from the OpMode
    public void setPIDF(double p, double i, double d, double f) {
        pid.setPIDF(p, i, d, f);
    }

    public void setReverse(boolean reverse) {
        if (reverse) {
            crServo.setDirection(CRServo.Direction.REVERSE);
        }
        else {
            crServo.setDirection(CRServo.Direction.FORWARD);
        }
    }

    /**
     * Get current angle from encoder with offset applied
     * @return Current angle in degrees (-180 to 180)
     */
    public double getAngle() {
        double rawAngle = (encoder.getVoltage() / 3.3) * 360.0;
        double offsetAngle = rawAngle - offset;
        return normalizeAngle(offsetAngle);
    }

    /**
     * Get the current target angle
     * @return Target angle in degrees
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Get the error between target and current position
     * @return Error in degrees
     */
    public double getError() {
        return normalizeAngle(targetAngle - getAngle());
    }

    private double normalizeAngle(double angle) {
        angle = (angle + 180.0) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    public double getVoltage() {
        return encoder.getVoltage() / 3.3;
    }

    /**
     * Get the last commanded power for debugging
     * @return Power value from -1.0 to 1.0
     */
    public double getLastPower() {
        return lastPower;
    }
}