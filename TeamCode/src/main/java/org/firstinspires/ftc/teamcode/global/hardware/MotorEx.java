package org.firstinspires.ftc.teamcode.global.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.global.constants;

import org.firstinspires.ftc.teamcode.global.control.PIDController;

public class MotorEx {

    private final DcMotorEx motor;
    private final HardwareUtil hardwareUtil;
    private PIDController pid;
    private double lastPower = 0.0;
    private final double THRESHOLD = constants.MOTOREX_POWER_THRESHOLD;

    public MotorEx(HardwareMap hwMap, HardwareUtil hardwareUtil, String name, double p, double i, double d, double f) {
        this.motor = hwMap.get(DcMotorEx.class, name);
        this.hardwareUtil = hardwareUtil;

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
        // Voltage compensation for battery droop
        power *= hardwareUtil.getVoltageMultiplier();

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