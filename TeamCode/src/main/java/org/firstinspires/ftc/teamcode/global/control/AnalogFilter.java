package org.firstinspires.ftc.teamcode.global.control;

public class AnalogFilter {
    private double lastEstimate = 0;
    private double alpha = 0;
    private boolean isInitialized = false;

    public AnalogFilter(double alpha) {
        this.alpha = alpha;
    }

    public double estimate(double rawAngle) {
        if (!isInitialized) {
            lastEstimate = rawAngle;
            isInitialized = true;
            return rawAngle;
        }

        // Handle Wrap-Around (Shortest Path Averaging)
        // This calculates the difference in the -180 to 180 range buh
        double diff = rawAngle - lastEstimate;

        // Normalize diff to -180 to 180 buh
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;

        // Apply the low pass filter to the difference.
        double currentEstimate = lastEstimate + (alpha * diff);

        lastEstimate = currentEstimate;
        return currentEstimate;
    }
}