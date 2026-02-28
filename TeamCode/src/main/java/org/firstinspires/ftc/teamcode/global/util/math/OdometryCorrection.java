package org.firstinspires.ftc.teamcode.global.util.math;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import java.util.ArrayList;

public class OdometryCorrection {
    private double scalarX;
    private double scalarY;
    private double scalarH;

    private final ArrayList<double[]> dataPoints = new ArrayList<>();

    public OdometryCorrection(double scalarX, double scalarY, double scalarH) {
        this.scalarX = scalarX;
        this.scalarY = scalarY;
        this.scalarH = scalarH;
    }

    public OdometryCorrection(double linearScalar) {
        this(linearScalar, linearScalar, 1.0);
    }

    public SparkFunOTOS.Pose2D correct(SparkFunOTOS.Pose2D raw) {
        return new SparkFunOTOS.Pose2D(
                raw.x * scalarX,
                raw.y * scalarY,
                raw.h * scalarH
        );
    }

    public void addDataPoint(double actual, double reported) {
        dataPoints.add(new double[]{actual, reported});
    }

    public void clearDataPoints() {
        dataPoints.clear();
    }

    public int getDataPointCount() {
        return dataPoints.size();
    }

    public double computeBestFit() {
        if (dataPoints.isEmpty()) return 1.0;

        // Least squares for y = mx (no intercept):
        // m = Σ(xi * yi) / Σ(xi²)  where x = reported, y = actual
        double sumXY = 0;
        double sumXX = 0;
        for (double[] pt : dataPoints) {
            double actual = pt[0];
            double reported = pt[1];
            sumXY += reported * actual;
            sumXX += reported * reported;
        }

        if (sumXX < 0.0001) return 1.0;
        return sumXY / sumXX;
    }

    public double computeMeanScalar() {
        if (dataPoints.isEmpty()) return 1.0;

        double sum = 0;
        int count = 0;
        for (double[] pt : dataPoints) {
            if (Math.abs(pt[1]) > 0.001) {
                sum += pt[0] / pt[1];
                count++;
            }
        }
        return count > 0 ? sum / count : 1.0;
    }

    public String getSummary() {
        StringBuilder sb = new StringBuilder();
        sb.append("Points: ").append(dataPoints.size()).append("\n");

        for (int i = 0; i < dataPoints.size(); i++) {
            double actual = dataPoints.get(i)[0];
            double reported = dataPoints.get(i)[1];
            double ratio = Math.abs(reported) > 0.001 ? actual / reported : 0;
            sb.append(String.format("  #%d: %.1f actual / %.1f reported = %.4f\n", i + 1, actual, reported, ratio));
        }

        sb.append(String.format("Mean scalar:      %.4f\n", computeMeanScalar()));
        sb.append(String.format("Best-fit scalar:  %.4f\n", computeBestFit()));
        return sb.toString();
    }

    public static double calibrate(double actualDist, double reportedDist) {
        if (Math.abs(reportedDist) < 0.001) return 1.0;
        return actualDist / reportedDist;
    }

    public void setScalarX(double scalarX) { this.scalarX = scalarX; }
    public void setScalarY(double scalarY) { this.scalarY = scalarY; }
    public void setScalarH(double scalarH) { this.scalarH = scalarH; }
    public double getScalarX() { return scalarX; }
    public double getScalarY() { return scalarY; }
    public double getScalarH() { return scalarH; }
}

