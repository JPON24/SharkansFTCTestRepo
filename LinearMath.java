package com.sharklib.core.util.math;

public class LinearMath {

    /**
     * Normalizes an angle to be between -180 and 180 degrees.
     * Just for regular servos and things
     */
    public static double angleWrap(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }

    /**
     * Clamps a value between a minimum and maximum.
     * Use this for Servo positions (0-1) or Motor Powers (-1 to 1).
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Linear Interpolation (Lerp).
     * @param a Start value
     * @param b End value
     * @param t Interpolation factor (0.0 to 1.0)
     */
    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    /**
     * Maps a value from one range to another.
     * Example: Map trigger pull (0.0 to 1.0) to servo angle (0.2 to 0.8)
     */
    public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return outMin + (outMax - outMin) * ((value - inMin) / (inMax - inMin));
    }

    /**
     * Checks if two numbers are effectively equal (within a small margin of error).
     */
    public static boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < 1e-6;
    }
}