package com.sharklib.core.hardware;

import com.sharklib.core.control.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PMotor {

    private final DcMotorEx motor;
    private PIDController pid;
    private double lastPower = 0.0;
    private final double THRESHOLD = 0.005; // Slightly tighter threshold for precision

    public PMotor(HardwareMap hwMap, String name, double p, double i, double d, double f) {
        this.motor = hwMap.get(DcMotorEx.class, name);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        this.pid = new PIDController(p, i, d, f);
    }

    /**
     * Updates the motor to reach a target encoder position.
     * Call this inside your loop()!
     */
    public void setPosition(double target) {
        // Fetch the current position automatically
        double current = motor.getCurrentPosition();

        // Calculate the necessary power via PID
        double power = pid.update(target, current);

        setPower(power);
    }

    public void setPower(double power) {
        // Caching to prevent USB bus congestion
        if (Math.abs(power - lastPower) > THRESHOLD || (power == 0 && lastPower != 0)) {
            motor.setPower(power);
            lastPower = power;
        }
    }

    // Helper methods for SharkLib
    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void setPIDF(double p, double i, double d, double f) {
        pid.setPIDF(p, i, d, f);
    }

    public void setReverse(boolean reverse) {
        if (reverse) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }
        else {
            motor.setDirection(DcMotorEx.Direction.FORWARD);
        }

    }
}