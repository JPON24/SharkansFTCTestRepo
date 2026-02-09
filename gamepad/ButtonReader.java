package com.sharklib.core.util.gamepad;

public class ButtonReader {
    private boolean lastState = false;
    private boolean currentState = false;
    private boolean toggleState = false;

    /**
     * This method needs to be called every loop.
     * It updates the state and handles the toggle logic.
     */
    public void update(boolean isButtonDown) {
        lastState = currentState;
        currentState = isButtonDown;

        // If a new press happens, flip the internal toggle switch
        if (wasJustPressed()) {
            toggleState = !toggleState;
        }
    }

    /**
     * True ONLY on the exact frame the button is clicked.
     */
    public boolean wasJustPressed() {
        return currentState && !lastState;
    }

    /**
     * True ONLY on the exact frame the button is released.
     */
    public boolean wasJustReleased() {
        return !currentState && lastState;
    }

    /**
     * Returns the "Light Switch" state (On/Off).
     */
    public boolean getToggle() {
        return toggleState;
    }

    /**
     * Standard check: is the button currently held down?
     */
    public boolean isDown() {
        return currentState;
    }

    /**
     * Allows you to manually reset the toggle (e.g., force claw closed)
     */
    public void setToggle(boolean state) {
        this.toggleState = state;
    }
}