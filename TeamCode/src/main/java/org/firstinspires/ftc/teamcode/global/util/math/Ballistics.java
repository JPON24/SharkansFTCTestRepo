package org.firstinspires.ftc.teamcode.global.util.math;

public class Ballistics {

    public static final double GRAVITY = 386.09; // inches/sec^2

    /**
     * Calculates the initial velocity required to hit a target.
     * Note: Not used in VirtualGoalShooter if using tables for RPM.
     */
    public static double calculateVelocity(double distanceToTarget, double heightDiff, double launchAngleDeg) {
        double theta = Math.toRadians(launchAngleDeg);
        double x = distanceToTarget;
        double y = heightDiff;
        double g = GRAVITY;

        double numerator = g * Math.pow(x, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y);

        if (denominator <= 0) return 0;

        return Math.sqrt(numerator / denominator);
    }

    /**
     * Calculates Time of Flight based on physical parameters.
     * Used for Motion Compensation.
     */
    public static double calculateTimeOfFlight(double distanceToTarget, double velocity, double launchAngleDeg) {
        double theta = Math.toRadians(launchAngleDeg);
        double vx = velocity * Math.cos(theta);
        if (Math.abs(vx) < 0.001) return 0; // Avoid divide by zero
        return distanceToTarget / vx;
    }

    public static double toRPM(double velocityInSec, double wheelRadiusInches, double gearRatio) {
        double circumference = 2 * Math.PI * wheelRadiusInches;
        double wheelRPM = (velocityInSec * 60) / circumference;
        return wheelRPM * gearRatio;
    }

    /**
     * Converts Motor RPM to Linear Velocity (in/sec).
     * Necessary to use Table RPMs in Physics calculations.
     */
    public static double toLinearVelocity(double motorRPM, double wheelRadiusInches, double gearRatio) {
        double wheelRPM = motorRPM / gearRatio;
        double circumference = 2 * Math.PI * wheelRadiusInches;
        // RPM / 60 = RPS. RPS * Circumference = in/sec
        return (wheelRPM / 60.0) * circumference;
    }
}