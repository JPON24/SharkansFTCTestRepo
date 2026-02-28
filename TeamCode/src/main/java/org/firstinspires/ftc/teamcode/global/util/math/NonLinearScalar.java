package org.firstinspires.ftc.teamcode.global.util.math;

import java.util.Map;
import java.util.TreeMap;

public class NonLinearScalar {
    private final TreeMap<Double, Double> references = new TreeMap<>();

    public NonLinearScalar() {}

    public void addReference(double input, double output) {
        references.put(input, output);
    }

    /**
     * Figure it out Jacob... Too lazy to explain... I'm hungryyyy
     */
    public double evaluate(double input) {
        if (references.isEmpty()) return 0;
        if (references.size() == 1) return references.firstEntry().getValue();

        if (references.containsKey(input)) return references.get(input);

        Map.Entry<Double, Double> lower = references.floorEntry(input);
        Map.Entry<Double, Double> upper = references.ceilingEntry(input);

        if (lower == null) {
            Map.Entry<Double, Double> first = references.firstEntry();
            Map.Entry<Double, Double> second = references.higherEntry(first.getKey());
            double slope = (second.getValue() - first.getValue()) / (second.getKey() - first.getKey());
            return first.getValue() + slope * (input - first.getKey());
        }

        if (upper == null) {
            Map.Entry<Double, Double> last = references.lastEntry();
            Map.Entry<Double, Double> secondLast = references.lowerEntry(last.getKey());
            double slope = (last.getValue() - secondLast.getValue()) / (last.getKey() - secondLast.getKey());
            return last.getValue() + slope * (input - last.getKey());
        }

        // Within range — linear interpolation
        double t = (input - lower.getKey()) / (upper.getKey() - lower.getKey());
        return LinearMath.lerp(lower.getValue(), upper.getValue(), t);
    }

    public void clear() {
        references.clear();
    }

    public int size() {
        return references.size();
    }
}
