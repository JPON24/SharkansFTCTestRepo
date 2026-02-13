package org.firstinspires.ftc.teamcode.global;

public class Vector2D {
    public final double x;
    public final double y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Calculate the magnitude (length) of the vector.
     * Equivalent to Math.hypot(x, y).
     */
    public double magnitude() {
        return Math.hypot(x, y);
    }

    /**
     * Calculate the angle of the vector in radians.
     */
    public double angle() {
        return Math.atan2(y, x);
    }

    /**
     * Rotates the vector by a specific angle.
     * @param angleRadians The angle to rotate by (counter-clockwise).
     * @return New rotated Vector2d
     */
    public Vector2D rotate(double angleRadians) {
        double newX = x * Math.cos(angleRadians) - y * Math.sin(angleRadians);
        double newY = x * Math.sin(angleRadians) + y * Math.cos(angleRadians);
        return new Vector2D(newX, newY);
    }

    /**
     * Vector Addition
     */
    public Vector2D add(Vector2D other) {
        return new Vector2D(this.x + other.x, this.y + other.y);
    }

    /**
     * Vector Subtraction
     */
    public Vector2D subtract(Vector2D other) {
        return new Vector2D(this.x - other.x, this.y - other.y);
    }

    /**
     * Scalar Multiplication (Scale the vector)
     */
    public Vector2D times(double scalar) {
        return new Vector2D(this.x * scalar, this.y * scalar);
    }

    /**
     * Returns a string representation (useful for telemetry).
     */
    @Override
    public String toString() {
        return String.format("(%.2f, %.2f)", x, y);
    }
}