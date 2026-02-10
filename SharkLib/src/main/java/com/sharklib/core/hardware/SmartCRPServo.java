package com.sharklib.core.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sharklib.core.control.PIDController;
import com.sharklib.core.util.math.LinearMath;

public class SmartCRPServo {

    private CRServo crServo;
    private PIDController pid;
    private AnalogInput encoder;

    private double lastPower = 0.0;
    public double THRESHOLD = 0.01;

    /**
     * Constructor
     * @param hwMap The hardware map
     * @param servoName The name of the CR Servo in config
     * @param encoderName The name of the Analog Encoder in config
     */
    public SmartCRPServo(HardwareMap hwMap, String servoName, String encoderName, double kP, double kI, double kD, double kF) {
        this.crServo = hwMap.get(CRServo.class, servoName);
        this.encoder = hwMap.get(AnalogInput.class, encoderName);

        // Initialize PID with default tuning (You MUST tune these!)
        this.pid = new PIDController(kP, kI, kD, kF);
    }

    /**
     * Call this in your loop to force the servo to a specific angle
     * @param targetAngle The target angle (e.g., 0 to 360 degrees)
     */
    public void setPosition(double targetAngle) {
        double currentVoltage = encoder.getVoltage();
        double currentAngle = (currentVoltage / 3.3) * 360.0;

        targetAngle = LinearMath.angleWrap(targetAngle);
        double power = pid.update(targetAngle, currentAngle);

        setPower(power);
    }

    /**
     * Standard manual power control
     */
    public void setPower(double power) {
        // Safety Clamp
        if (power > 1.0) power = 1.0;
        if (power < -1.0) power = -1.0;

        // Caching Logic
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

    // Debug helper
    public double getAngle() {
        return (encoder.getVoltage() / 3.3) * 360.0;
    }
}