package com.sharklib.core.util.math;

public class Ballistics {

    public static final double GRAVITY = 386.09; // its in inchs/sec^2



    /**
     * Calculates the initial velocity (RPM or in/sec) required to hit a target.
     * Uses standard Kinematic projectile motion equations.
     *
     * @param distanceToTarget Horizontal distance to target (inches)
     * @param heightDiff       Target Height - Shooter Height (inches)
     * @param launchAngleDeg   The physical angle of the shooter
     * @return Required launch velocity in inches/sec
     */
    public static double calculateVelocity(double distanceToTarget, double heightDiff, double launchAngleDeg) {
        double theta = Math.toRadians(launchAngleDeg);
        double x = distanceToTarget;
        double y = heightDiff;
        double g = GRAVITY;

        // Physics Formula: v = sqrt( (g*x^2) / (2 * cos^2(theta) * (x*tan(theta) - y)) )

        double numerator = g * Math.pow(x, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y);

        // Safety: If denominator is negative, the shot is physically impossible
        // (Target is too high for the angle)
        if (denominator <= 0) return 0;

        return Math.sqrt(numerator / denominator);
    }

    /**
     * Calculates how long the game element will be in the air.
     * Latency for shooting while moving buh
     *
     * @param distanceToTarget Horizontal distance (inches)
     * @param velocity         Launch velocity (inches/sec)
     * @param launchAngleDeg   Shooter angle
     * @return Time in seconds
     */
    public static double calculateTimeOfFlight(double distanceToTarget, double velocity, double launchAngleDeg) {
        double theta = Math.toRadians(launchAngleDeg);
        double vx = velocity * Math.cos(theta);
        return distanceToTarget / vx;
    }

    /**
     * Converts Linear Velocity (in/sec) to Motor RPM.
     */
    public static double toRPM(double velocityInSec, double wheelRadiusInches, double gearRatio) {
        double circumference = 2 * Math.PI * wheelRadiusInches;
        double wheelRPM = (velocityInSec * 60) / circumference;
        return wheelRPM * gearRatio;
    }
}