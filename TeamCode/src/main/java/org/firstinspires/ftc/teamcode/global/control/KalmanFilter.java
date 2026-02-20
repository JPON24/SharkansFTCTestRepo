package org.firstinspires.ftc.teamcode.global.control;

public class KalmanFilter {
    private double Q; // Process noise covariance (How much the system changes)
    private double R; // Measurement noise covariance (How noisy the sensor is)
    private double P = 1; // Estimation error covariance
    private double K; // Kalman gain
    private double x; // Value estimate (The "Clean" Voltage)

    /**
     * @param Q Process noise (Keep this small, e.g., 0.001)
     * @param R Measurement noise (Keep this larger, e.g., 0.1)
     */
    public KalmanFilter(double Q, double R) {
        this.Q = Q;
        this.R = R;
        this.x = 0; // Initial guess
    }

    /**
     * Update the filter with a new raw measurement
     * @param measurement The raw, noisy value (e.g., voltageSensor.getVoltage())
     * @return The clean, filtered value
     */
    public double filter(double measurement) {
        P = P + Q;

        K = P / (P + R);
        x = x + K * (measurement - x);
        P = (1 - K) * P;

        return x;
    }

    public void setEstimate(double estimate) {
        this.x = estimate;
    }
}