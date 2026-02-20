package org.firstinspires.ftc.teamcode.global.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.global.constants;
import org.firstinspires.ftc.teamcode.global.control.KalmanFilter;
import org.firstinspires.ftc.teamcode.global.util.math.LinearMath;
import org.firstinspires.ftc.teamcode.global.control.PIDController;
import org.firstinspires.ftc.teamcode.global.hardware.HardwareUtil;

public class CRServoEx {

    private KalmanFilter filter;
    private double lastFilteredAngle = 0;
    private boolean filterInitialized = false;

    private HardwareUtil hardwareUtil;

    private CRServo crServo;
    private PIDController pid;
    private AnalogInput encoder;

    private double lastPower = 0.0;
    private double targetAngle = 0.0;  // Store the target angle
    public double THRESHOLD = constants.CRSERVO_POWER_THRESHOLD;

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
     * @param alpha Unused (kept for API compatibility) — Kalman Q/R are set in constants
     * @param offset The encoder offset in degrees
     */
    public CRServoEx(HardwareMap hwMap, HardwareUtil hardwareUtil, String servoName, String encoderName, double kP, double kI, double kD, double kF, double alpha, double offset) {
        this.crServo = hwMap.get(CRServo.class, servoName);
        this.encoder = hwMap.get(AnalogInput.class, encoderName);

        this.hardwareUtil = hardwareUtil;
        this.pid = new PIDController(kP, kI, kD, kF);
        this.filter = new KalmanFilter(constants.CRSERVO_KALMAN_Q, constants.CRSERVO_KALMAN_R);

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
        double currentAngle = getAngle(); // I AMM TRUUUUTH!!! Halo reference

        double wrappedError = LinearMath.angleWrap(targetAngle - currentAngle);

        double power = pid.updateWithError(wrappedError, targetAngle);
        power *= hardwareUtil.getVoltageMultiplier();

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
        double rawAngle = (encoder.getVoltage() / constants.ANALOG_VOLTAGE_REF) * 360.0;

        // Seed Kalman filter on first read to avoid convergence lag
        if (!filterInitialized) {
            filter.setEstimate(rawAngle);
            lastFilteredAngle = rawAngle;
            filterInitialized = true;
        }

        // Handle wrap-around: unwrap relative to last filtered value
        // so the Kalman filter sees smooth continuous input
        double diff = rawAngle - lastFilteredAngle;
        if (diff > 180) diff -= 360;
        if (diff < -180) diff += 360;
        double unwrapped = lastFilteredAngle + diff;

        double filteredAngle = filter.filter(unwrapped);
        lastFilteredAngle = filteredAngle;

        double offsetAngle = filteredAngle - offset;
        return LinearMath.angleWrap(offsetAngle);
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
        return LinearMath.angleWrap(targetAngle - getAngle());
    }



    public double getVoltage() {
        return encoder.getVoltage() / constants.ANALOG_VOLTAGE_REF;
    }

    /**
     * Get the last commanded power for debugging
     * @return Power value from -1.0 to 1.0
     */
    public double getLastPower() {
        return lastPower;
    }
}