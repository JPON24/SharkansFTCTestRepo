package org.firstinspires.ftc.teamcode.global.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoEx {
    private Servo servo;
    private double lastPosition = -1.0; // Start at -1 so the first command always works
    private final double THRESHOLD = 0.001; // Servos are precise, so we use a small threshold

    public ServoEx(HardwareMap hwMap, String Name) {
        this.servo = hwMap.get(Servo.class, Name);
    }

    public void setPosition(double position) {
        if (position < 0.0) position = 0.0;
        if (position > 1.0) position = 1.0;

        if (Math.abs(position - lastPosition) > THRESHOLD) {
            servo.setPosition(position);
            lastPosition = position;
        }
    }

    public double getPosition() {
        return lastPosition;
    }

    public void setReverse(boolean isReversed) {
        if (isReversed) {
            servo.setDirection(Servo.Direction.REVERSE);
        }
        else {
            servo.setDirection(Servo.Direction.FORWARD);
        }
    }

}
