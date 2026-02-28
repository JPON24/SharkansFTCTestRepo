package org.firstinspires.ftc.teamcode.global.util.math;

import java.util.Map;
import java.util.TreeMap;

public class InterpLUT {
    // TreeMap automatically sorts data by the "Key"
    private final TreeMap<Double, Double> table = new TreeMap<>();
    private String name;

    public InterpLUT(String name) {
        this.name = name;
    }

    /**
     * Add a calibration point.
     * @param input
     * @param output
     */
    public void add(double input, double output) {
        table.put(input, output);
    }

    /**
     * Get the calculated value for a specific input.
     */
    public double get(double input) {
        if (table.isEmpty()) return 0;

        // Check for exact match
        if (table.containsKey(input)) return table.get(input);

        // Find the closest known points below and above
        Map.Entry<Double, Double> lower = table.floorEntry(input);
        Map.Entry<Double, Double> upper = table.ceilingEntry(input);

        // Below range — extrapolate using first two points
        if (lower == null) {
            Map.Entry<Double, Double> first = table.firstEntry();
            Map.Entry<Double, Double> second = table.higherEntry(first.getKey());
            if (second == null) return first.getValue();
            double slope = (second.getValue() - first.getValue()) / (second.getKey() - first.getKey());
            return first.getValue() + slope * (input - first.getKey());
        }

        // Above range — extrapolate using last two points
        if (upper == null) {
            Map.Entry<Double, Double> last = table.lastEntry();
            Map.Entry<Double, Double> secondLast = table.lowerEntry(last.getKey());
            if (secondLast == null) return last.getValue();
            double slope = (last.getValue() - secondLast.getValue()) / (last.getKey() - secondLast.getKey());
            return last.getValue() + slope * (input - last.getKey());
        }

        // Linear Interpolation: Calculate the point on the line between the two data points
        double t = (input - lower.getKey()) / (upper.getKey() - lower.getKey());
        return LinearMath.lerp(lower.getValue(), upper.getValue(), t);
    }

    public void clear() {
        table.clear();
    }
}