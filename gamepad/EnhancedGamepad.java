package com.sharklib.core.util.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class EnhancedGamepad {
    private Gamepad gamepad;

    /**
        We create readers for the most used buttons
     */
    public ButtonReader a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right, lb, rb;

    public EnhancedGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
        a = new ButtonReader();
        b = new ButtonReader();
        x = new ButtonReader();
        y = new ButtonReader();
        dpad_up = new ButtonReader();
        dpad_down = new ButtonReader();
        dpad_left = new ButtonReader();
        dpad_right = new ButtonReader();
        lb = new ButtonReader();
        rb = new ButtonReader();
    }

    /**
     * Call this ONCE at the start of your loop()
     */
    public void update() {
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        dpad_up.update(gamepad.dpad_up);
        dpad_down.update(gamepad.dpad_down);
        dpad_left.update(gamepad.dpad_left);
        dpad_right.update(gamepad.dpad_right);
        lb.update(gamepad.left_bumper);
        rb.update(gamepad.right_bumper);
    }

    /**
     * You can still access raw joystick values
     */

    public double left_stick_x() {return gamepad.left_stick_x; }
    public double left_stick_y() { return gamepad.left_stick_y; }
    public double right_stick_x() { return gamepad.right_stick_x; }
    public double right_stick_y( ) {return gamepad.right_stick_y; }
}
