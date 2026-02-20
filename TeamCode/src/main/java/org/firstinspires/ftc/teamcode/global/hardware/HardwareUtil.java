package org.firstinspires.ftc.teamcode.global.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.global.constants;
import org.firstinspires.ftc.teamcode.global.control.KalmanFilter;

import java.util.List;

public class HardwareUtil {
    private VoltageSensor voltageSensor;
    private List<LynxModule> allHubs;

    private KalmanFilter voltageFilter;

    private double baselineVoltage = constants.VOLTAGE_BASELINE_DEFAULT;
    private double cachedVoltage = constants.VOLTAGE_CACHED_DEFAULT;

    public HardwareUtil(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);

        // Auto-Bulk Read Mode (Performance)
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        voltageFilter = new KalmanFilter(constants.VOLTAGE_KALMAN_Q, constants.VOLTAGE_KALMAN_R);

        double sum = 0;
        for (int i = 0; i < constants.VOLTAGE_SAMPLE_COUNT; i++) {
            sum += voltageSensor.getVoltage();
        }
        baselineVoltage = sum / constants.VOLTAGE_SAMPLE_COUNT;

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
        if (cachedVoltage < constants.VOLTAGE_SAFETY_MIN) return 1.0;

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