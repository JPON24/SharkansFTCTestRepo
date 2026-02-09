package com.sharklib.core.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    private final DcMotor Motor;
    private double lastPower = 0.0;
    private final double THRESHOLD = 0.01; // Only update if change is > 1%

    public Motor(HardwareMap hwMap, String name) {
        this.Motor = hwMap.get(DcMotor.class, name);
    }

    public void setPower(double power) {
        // Caching
        if (Math.abs(power - lastPower) > THRESHOLD) {
            Motor.setPower(power);
            lastPower = power;
        }
    }

    public void setReverse(boolean reverse) {
        if (reverse) {
            Motor.setDirection(DcMotor.Direction.REVERSE);
        }
        else {
            Motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public DcMotor getRawMotor() {
        return Motor;
    }
}
