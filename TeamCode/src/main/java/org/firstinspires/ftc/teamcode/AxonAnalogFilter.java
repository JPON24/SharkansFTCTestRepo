package org.firstinspires.ftc.teamcode;

public class AxonAnalogFilter {
    private double lastEstimate = 0;
    private double alpha = 0.3;
    private boolean isInitialized = false;

    public AxonAnalogFilter(double alpha) {
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