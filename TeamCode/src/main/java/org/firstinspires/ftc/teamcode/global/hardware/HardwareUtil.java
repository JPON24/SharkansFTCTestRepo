package org.firstinspires.ftc.teamcode.global.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.global.control.KalmanFilter;

import java.util.List;

public class HardwareUtil {
    private VoltageSensor voltageSensor;
    private List<LynxModule> allHubs;

    private KalmanFilter voltageFilter;

    private double baselineVoltage = 11.5; // Default safe value
    private double cachedVoltage = 12.0;

    public HardwareUtil(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);

        // Auto-Bulk Read Mode (Performance)
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Q=0.001 (Slow changes), R=0.5 (Noisy sensor)
        voltageFilter = new KalmanFilter(0.001, 0.5);

        // We read it 10 times to average out any startup noise
        double sum = 0;
        for (int i = 0; i < 10; i++) {
            sum += voltageSensor.getVoltage();
        }
        baselineVoltage = sum / 10.0;

        // Seed the filter with this clean baseline
        voltageFilter.setEstimate(baselineVoltage);
        cachedVoltage = baselineVoltage;
    }

    /**
     * Call this at the START of every loop()
     */
    public void update() {
        // Clear Hub Cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Filter the voltage
        double rawVoltage = voltageSensor.getVoltage();
        cachedVoltage = voltageFilter.filter(rawVoltage);
    }

    /**
     * Returns the multiplier based on the STARTING voltage of this specific match.
     */
    public double getVoltageMultiplier() {
        // Prevent division by zero or massive spikes if battery dies
        if (cachedVoltage < 5.0) return 1.0;

        // Example: Started at 13V, now at 11V.
        // Result = 13.0 / 11.0 = 1.18 (18% boost)
        return baselineVoltage / cachedVoltage;
    }

    public double getVoltage() {
        return cachedVoltage;
    }

    public double getBaseline() {
        return baselineVoltage;
    }
}