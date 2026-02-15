package org.firstinspires.ftc.teamcode.global.util.gamepad;

public class ButtonReader {
    private boolean lastState = false;
    private boolean currentState = false;
    private boolean toggleState = false;

    // Standard update for booleans (A, B, X, Y, Bumpers)
    public void update(boolean isButtonDown) {
        lastState = currentState;
        currentState = isButtonDown;

        if (wasJustPressed()) {
            toggleState = !toggleState;
        }
    }

    // Overloaded update for Triggers (LT, RT)
    public void update(float triggerValue) {
        // Convert the float to a boolean using a 0.4 threshold
        this.update(triggerValue > 0.4f);
    }

    public boolean wasJustPressed() { return currentState && !lastState; }
    public boolean wasJustReleased() { return !currentState && lastState; }
    public boolean getToggle() { return toggleState; }
    public boolean isDown() { return currentState; }
    public void setToggle(boolean state) { this.toggleState = state; }
}