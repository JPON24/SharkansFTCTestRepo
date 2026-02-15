package org.firstinspires.ftc.teamcode.global.control;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {

    private double kP, kI, kD, kF;
    private double lastError = 0;
    private double integralSum = 0;
    private double maxIntegral = 180;

    ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        timer.reset();
    }

    /**
     * Standard update loop using target and current values.
     * Note: This does NOT handle angle wrapping. Use updateWithError for swerve.
     */
    public double update(double target, double current) {
        double error = target - current;
        return updateWithError(error, target);
    }

    /**
     * Accepts pre-calc error
     * Use this when you've already handled shortest-path math (angleWrap).
     * @param error The wrapped error (target - current)
     * @param target The original target (used only for Feedforward/kF)
     */
    public double updateWithError(double error, double target) {
        double dt = timer.seconds();
        if (dt < 0.001) dt = 0.001; // Prevent divide by zero/time jumps
        timer.reset();

        // Proportional Term
        double pTerm = error * kP;

        // Integral Term with Windup Protection
        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);

        // Anti-Windup: Reset integral if we cross the setpoint
        if (Math.signum(error) != Math.signum(lastError)) {
            integralSum = 0;
        }

        double iTerm = integralSum * kI;

        // Derivative Term (Rate of Change)
        double derivative = (error - lastError) / dt;
        double dTerm = derivative * kD;

        // Feedforward Term (Based on target position)
        double fTerm = target * kF;

        lastError = error;

        double output = pTerm + iTerm + dTerm + fTerm;

        return Range.clip(output, -1.0, 1.0);
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }

    public void setMaxIntegral(double max) {
        this.maxIntegral = max;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}