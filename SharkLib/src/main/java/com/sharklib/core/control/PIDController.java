package com.sharklib.core.control;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {

    private double kP, kI, kD, kF;
    private double lastError = 0;
    private double integralSum = 0;
    private double maxIntegral = 10.0;

    ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        timer.reset();
    }
    public double update(double target, double current) {
        double dt = timer.seconds();
        if (dt < 0.001) dt = 0.001;
        timer.reset();

        double error = target - current;

        double pTerm = error * kP;

        if (dt > 0) {
            integralSum += error * dt;
            integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
        }
        double iTerm = integralSum * kI;

        double derivative = 0;
        if (dt > 0) {
            derivative = (error - lastError) / dt;
        }
        double dTerm = derivative * kD;

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
}