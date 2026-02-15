package org.firstinspires.ftc.teamcode.global.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Motor {
    private final DcMotor Motor;
    private HardwareUtil hardwareUtil = null;
    private double lastPower = 0.0;
    private final double THRESHOLD = 0.001; // Only update if change is > 0.1%

    public Motor(HardwareMap hwMap, String name) {
        this.Motor = hwMap.get(DcMotor.class, name);
    }

    public Motor(HardwareMap hwMap, HardwareUtil hardwareUtil, String name) {
        this.Motor = hwMap.get(DcMotor.class, name);
        this.hardwareUtil = hardwareUtil;
    }

    public void setPower(double power) {
        // Voltage compensation for battery droop (if HardwareUtil is provided)
        if (hardwareUtil != null) {
            power *= hardwareUtil.getVoltageMultiplier();
        }

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
