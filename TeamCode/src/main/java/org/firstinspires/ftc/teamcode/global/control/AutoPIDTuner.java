package org.firstinspires.ftc.teamcode.global.control;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.global.constants;

import java.io.File;

/**
 * AutoPIDTuner — Automatic PID tuning using the Ziegler-Nichols method
 *
 * HOW IT WORKS:
 * 1. You give it a motor and a target position.
 * 2. It slowly increases kP (with kI=0, kD=0) until the system oscillates.
 * 3. The kP where oscillation is sustained = Ultimate Gain (Ku).
 * 4. The time for one full oscillation = Ultimate Period (Tu).
 * 5. It calculates kP, kI, kD using the Ziegler-Nichols formulas.
 * 6. Results are saved to a file on the robot so you can use them later.
 *
 * HOW TO USE:
 *   AutoPIDTuner tuner = new AutoPIDTuner("armMotor");
 *   tuner.setTarget(500);            // Encoder ticks to oscillate around
 *   tuner.setKPRange(0.001, 0.1);    // Min/max kP to try
 *   tuner.setSafetyLimits(0.5, 10);  // Max power, max seconds per step
 *
 *   // In your loop:
 *   tuner.update(motor.getCurrentPosition());
 *   motor.setPower(tuner.getOutput());
 *
 *   // Check results:
 *   if (tuner.getState() == TunerState.DONE) {
 *       double[] gains = tuner.getResults();  // [kP, kI, kD]
 *       tuner.saveResults();                  // Saves to file
 *   }
 *
 * LOADING SAVED RESULTS (in a different OpMode):
 *   double[] gains = AutoPIDTuner.loadResults("armMotor");
 *   if (gains != null) {
 *       myPID.setPIDF(gains[0], gains[1], gains[2], 0);
 *   }
 *
 * File path: /sdcard/FIRST/pid_tuning/[systemName].txt
 */
public class AutoPIDTuner {

    // STATE MACHINE

    public enum TunerState {
        IDLE,            // Not started yet
        INCREASING_KP,   // Ramping kP to find sustained oscillation
        MEASURING,       // Found oscillation — measuring the period (Tu)
        DONE,            // Tuning complete, results ready
        FAILED           // Could not find oscillation within kP range
    }

    private TunerState state = TunerState.IDLE;

    // CONFIGURATION (set these before starting)

    private String systemName;         // Used for file naming ("armMotor", "turret", etc.)
    private double target = 0;         // Position to oscillate around (encoder ticks or degrees)

    private double kpMin = constants.TUNER_KP_MIN_DEFAULT;
    private double kpMax = constants.TUNER_KP_MAX_DEFAULT;
    private double kpStep = constants.TUNER_KP_STEP_DEFAULT;

    private double maxPower = constants.TUNER_MAX_POWER_DEFAULT;
    private double stepTimeout = constants.TUNER_STEP_TIMEOUT_DEFAULT;

    // TUNING STATE (internal)

    private double currentKP;          // kP being tested right now
    private double output = 0;         // Motor power output

    // Oscillation detection
    private double lastError = 0;
    private int zeroCrossings = 0;     // Error sign changes = half-oscillations
    private double peakError = 0;      // Current peak amplitude
    private double prevPeakError = 0;  // Previous peak amplitude (for stability check)
    private boolean lastErrorPositive = true;

    private static final int CROSSINGS_TO_CONFIRM = constants.TUNER_CROSSINGS_CONFIRM;
    private static final double STABILITY_THRESHOLD = constants.TUNER_STABILITY_THRESHOLD;

    // Period measurement
    private ElapsedTime stepTimer = new ElapsedTime();
    private ElapsedTime periodTimer = new ElapsedTime();
    private double[] periodSamples = new double[constants.TUNER_PERIODS_TO_MEASURE];
    private int periodIndex = 0;
    private int measureCrossings = 0;

    private static final int PERIODS_TO_MEASURE = constants.TUNER_PERIODS_TO_MEASURE;

    // RESULTS

    private double ultimateGain = 0;   // Ku
    private double ultimatePeriod = 0; // Tu (seconds)
    private double resultKP = 0;
    private double resultKI = 0;
    private double resultKD = 0;

    // CONSTRUCTOR

    /**
     * @param systemName A name for this system (used for saving/loading results).
     *                   Example: "armMotor", "turretMotor", "slideMotor"
     */
    public AutoPIDTuner(String systemName) {
        this.systemName = systemName;
    }

    // CONFIGURATION METHODS

    /** Set the target position to oscillate around (encoder ticks or degrees) */
    public void setTarget(double target) {
        this.target = target;
    }

    /** Set the range of kP values to probe. Start small! */
    public void setKPRange(double min, double max, double step) {
        this.kpMin = min;
        this.kpMax = max;
        this.kpStep = step;
    }

    /** Safety limits. maxPower caps motor output. timeout = seconds per kP step. */
    public void setSafetyLimits(double maxPower, double stepTimeoutSeconds) {
        this.maxPower = maxPower;
        this.stepTimeout = stepTimeoutSeconds;
    }

    // START / RESET

    /** Begin the tuning process. Call this once, then call update() every loop. */
    public void start() {
        state = TunerState.INCREASING_KP;
        currentKP = kpMin;
        zeroCrossings = 0;
        peakError = 0;
        prevPeakError = 0;
        output = 0;
        stepTimer.reset();
    }

    /** Reset everything so you can tune again */
    public void reset() {
        state = TunerState.IDLE;
        currentKP = kpMin;
        zeroCrossings = 0;
        peakError = 0;
        prevPeakError = 0;
        output = 0;
        ultimateGain = 0;
        ultimatePeriod = 0;
        resultKP = 0;
        resultKI = 0;
        resultKD = 0;
        periodIndex = 0;
        measureCrossings = 0;
    }

    // MAIN UPDATE — Call this every loop!

    /**
     * Feed in the current position reading. Call every loop cycle.
     *
     * @param currentPosition The current encoder/sensor reading
     */
    public void update(double currentPosition) {
        if (state == TunerState.DONE || state == TunerState.FAILED || state == TunerState.IDLE) {
            output = 0;
            return;
        }

        double error = target - currentPosition;

        switch (state) {
            case INCREASING_KP:
                runIncreasingKP(error);
                break;

            case MEASURING:
                runMeasuring(error);
                break;
        }

        lastError = error;
    }

    // STATE: INCREASING_KP
    //
    // Applies P-only control at the current kP.
    // Detects zero-crossings (error changes sign = half oscillation).
    // Tracks peak amplitudes. If consecutive peaks are similar → sustained oscillation → found Ku!

    private void runIncreasingKP(double error) {
        // Apply P-only control
        output = error * currentKP;
        output = clamp(output, -maxPower, maxPower);

        // Detect zero-crossing
        // A zero-crossing happens when the error changes sign
        boolean currentPositive = error >= 0;

        if (currentPositive != lastErrorPositive && Math.abs(lastError) > constants.TUNER_ZERO_CROSS_MIN_ERROR) {
            zeroCrossings++;
            lastErrorPositive = currentPositive;

            // At each zero-crossing, we've completed a half-cycle.
            // Compare the peak from this half-cycle to the previous one.
            if (zeroCrossings > 1) {
                // Check if oscillation is stable
                // "Stable" means consecutive peaks are similar in size.
                // This is how we know we've found Ku.
                if (prevPeakError > 0) {
                    double ratio = Math.abs(peakError - prevPeakError)
                            / Math.max(peakError, prevPeakError);

                    // YOUR DESIGN DECISION:
                    // ratio < STABILITY_THRESHOLD means peaks are within 15%
                    // You could make this tighter (5%) for more precision
                    // or looser (25%) if your system is noisy
                    if (ratio < STABILITY_THRESHOLD && zeroCrossings >= CROSSINGS_TO_CONFIRM) {
                        // Found Ku! Move to measuring period.
                        ultimateGain = currentKP;
                        enterMeasuringState();
                        return;
                    }
                }
                prevPeakError = peakError;
            }

            // Reset peak tracking for the next half-cycle
            peakError = 0;
        }

        // Track the peak error in this half-cycle
        peakError = Math.max(peakError, Math.abs(error));

        // Timeout: this kP isn't causing oscillation, try a bigger one
        if (stepTimer.seconds() > stepTimeout) {
            currentKP += kpStep;
            zeroCrossings = 0;
            peakError = 0;
            prevPeakError = 0;
            stepTimer.reset();

            // If we've exceeded the max kP, give up
            if (currentKP > kpMax) {
                state = TunerState.FAILED;
                output = 0;
            }
        }
    }

    // STATE: MEASURING
    //
    // We found Ku. Now measure the oscillation period (Tu) by timing zero-crossings.
    // Two zero-crossings = one full period.

    private void enterMeasuringState() {
        state = TunerState.MEASURING;
        measureCrossings = 0;
        periodIndex = 0;
        periodTimer.reset();
        lastErrorPositive = lastError >= 0;
    }

    private void runMeasuring(double error) {
        // Keep applying the same kP that caused sustained oscillation
        output = error * ultimateGain;
        output = clamp(output, -maxPower, maxPower);

        // Track zero-crossings and time them
        boolean currentPositive = error >= 0;

        if (currentPositive != lastErrorPositive && Math.abs(lastError) > constants.TUNER_ZERO_CROSS_MIN_ERROR) {
            measureCrossings++;
            lastErrorPositive = currentPositive;

            // Every 2 zero-crossings = 1 full period
            if (measureCrossings % 2 == 0 && measureCrossings > 0) {
                double period = periodTimer.seconds();
                periodTimer.reset();

                if (periodIndex < PERIODS_TO_MEASURE) {
                    periodSamples[periodIndex] = period;
                    periodIndex++;
                }

                // Once we have enough samples, average them and calculate gains
                if (periodIndex >= PERIODS_TO_MEASURE) {
                    ultimatePeriod = averagePeriods();
                    calculateZieglerNichols();
                    state = TunerState.DONE;
                    output = 0;
                }
            }
        }
    }

    private double averagePeriods() {
        double sum = 0;
        for (int i = 0; i < PERIODS_TO_MEASURE; i++) {
            sum += periodSamples[i];
        }
        return sum / PERIODS_TO_MEASURE;
    }

    // ZIEGLER-NICHOLS CALCULATION
    //
    //  Controller | kP         | kI           | kD
    //  -----------+------------+--------------+-------------------
    //  P only     | 0.50 * Ku  | 0            | 0
    //  PI         | 0.45 * Ku  | 0.54 * Ku/Tu | 0
    //  PID        | 0.60 * Ku  | 1.2  * Ku/Tu | 0.075 * Ku * Tu
    //
    // We use the full PID row. You can tweak these multipliers if
    // the results are too aggressive (common for FTC — try halving kI).

    private void calculateZieglerNichols() {
        resultKP = constants.TUNER_ZN_KP_MULT * ultimateGain;
        resultKI = constants.TUNER_ZN_KI_MULT * ultimateGain / ultimatePeriod;
        resultKD = constants.TUNER_ZN_KD_MULT * ultimateGain * ultimatePeriod;
    }

    // FILE PERSISTENCE

    private static final String SAVE_DIR = "pid_tuning";

    /**
     * Save tuning results to a file on the robot.
     * File: /sdcard/FIRST/pid_tuning/[systemName].txt
     */
    public void saveResults() {
        if (state != TunerState.DONE) return;

        String data = String.format(
                "kP=%.6f\nkI=%.6f\nkD=%.6f\nKu=%.6f\nTu=%.6f\ntarget=%.1f\n",
                resultKP, resultKI, resultKD, ultimateGain, ultimatePeriod, target
        );

        File dir = new File(AppUtil.ROBOT_DATA_DIR, SAVE_DIR);
        if (!dir.exists()) dir.mkdirs();

        File file = new File(dir, systemName + ".txt");
        ReadWriteFile.writeFile(file, data);
    }

    /**
     * Load previously saved tuning results from another OpMode.
     *
     * @param systemName The name used when saving (e.g., "armMotor")
     * @return double[3] = {kP, kI, kD}, or null if no saved results exist
     */
    public static double[] loadResults(String systemName) {
        File file = new File(
                new File(AppUtil.ROBOT_DATA_DIR, SAVE_DIR),
                systemName + ".txt"
        );

        if (!file.exists()) return null;

        try {
            String data = ReadWriteFile.readFile(file);
            String[] lines = data.split("\n");
            double kP = 0, kI = 0, kD = 0;

            for (String line : lines) {
                String[] parts = line.split("=");
                if (parts.length != 2) continue;

                switch (parts[0].trim()) {
                    case "kP": kP = Double.parseDouble(parts[1].trim()); break;
                    case "kI": kI = Double.parseDouble(parts[1].trim()); break;
                    case "kD": kD = Double.parseDouble(parts[1].trim()); break;
                }
            }
            return new double[]{ kP, kI, kD };
        } catch (Exception e) {
            return null;
        }
    }

    // GETTERS — Use these for telemetry

    /** The motor power output. Apply this to your motor each loop. */
    public double getOutput() { return output; }

    /** Current tuner state */
    public TunerState getState() { return state; }

    /** The kP currently being tested */
    public double getCurrentKP() { return currentKP; }

    /** How many zero-crossings detected so far in this kP step */
    public int getZeroCrossings() { return zeroCrossings; }

    /** The current peak error amplitude */
    public double getPeakError() { return peakError; }

    /** Final results: [kP, kI, kD]. Only valid when state == DONE. */
    public double[] getResults() { return new double[]{ resultKP, resultKI, resultKD }; }

    /** Ultimate gain (Ku). Only valid when state == DONE or MEASURING. */
    public double getUltimateGain() { return ultimateGain; }

    /** Ultimate period (Tu) in seconds. Only valid when state == DONE. */
    public double getUltimatePeriod() { return ultimatePeriod; }

    /** The system name used for file storage */
    public String getSystemName() { return systemName; }

    // UTILITY

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
