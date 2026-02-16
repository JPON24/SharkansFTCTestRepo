package org.firstinspires.ftc.teamcode.global;

/*
 * AutoPIDTuner Example OpMode
 *
 * This demonstrates how to tune a motor's PID on the fly,
 * save the results, and load them in your real OpModes.
 *
 * CONTROLS:
 *   A button  → Start tuning
 *   B button  → Save results to file
 *   X button  → Load saved results and test them
 *   Y button  → Reset tuner
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.global.control.AutoPIDTuner;
import org.firstinspires.ftc.teamcode.global.control.PIDController;

@TeleOp(name = "⚙️ Auto PID Tuner", group = "Tuning")
public class AutoPIDTunerOpMode extends OpMode {

    // The motor to tune
    DcMotorEx motor;

    // The tuner
    AutoPIDTuner tuner;

    // For testing loaded results
    PIDController testPID;
    boolean testMode = false;
    double testTarget = 0;

    @Override
    public void init() {
        // 1. Set up the motor
        motor = hardwareMap.get(DcMotorEx.class, "armMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Create the tuner with a name (used for saving)
        tuner = new AutoPIDTuner("armMotor");

        // 3. Configure it
        tuner.setTarget(300);                     // Oscillate around 300 ticks
        tuner.setKPRange(0.001, 0.05, 0.001);    // Try kP from 0.001 to 0.05
        tuner.setSafetyLimits(0.5, 8.0);          // Max 50% power, 8s per step

        // 4. Set up test PID (for after tuning)
        testPID = new PIDController(0, 0, 0, 0);

        telemetry.addData("Ready", "Press A to start tuning");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ── Controls ──

        // A = Start tuning
        if (gamepad1.a && tuner.getState() == AutoPIDTuner.TunerState.IDLE) {
            tuner.start();
            testMode = false;
        }

        // B = Save results
        if (gamepad1.b && tuner.getState() == AutoPIDTuner.TunerState.DONE) {
            tuner.saveResults();
        }

        // X = Load saved results and test them
        if (gamepad1.x) {
            double[] saved = AutoPIDTuner.loadResults("armMotor");
            if (saved != null) {
                testPID.setPIDF(saved[0], saved[1], saved[2], 0);
                testTarget = 300;
                testMode = true;
            }
        }

        // Y = Reset
        if (gamepad1.y) {
            tuner.reset();
            testMode = false;
            motor.setPower(0);
        }


        // ── Run the tuner or test mode ──

        if (testMode) {
            // Test the tuned PID
            double power = testPID.update(testTarget, motor.getCurrentPosition());
            motor.setPower(power);

            telemetry.addLine("═══ TEST MODE ═══");
            telemetry.addData("Target", "%.0f", testTarget);
            telemetry.addData("Current", "%d", motor.getCurrentPosition());
            telemetry.addData("Error", "%.1f", testTarget - motor.getCurrentPosition());

        } else {
            // Feed the tuner
            tuner.update(motor.getCurrentPosition());
            motor.setPower(tuner.getOutput());

            // ── Telemetry ──

            telemetry.addLine("═══ AUTO PID TUNER ═══");
            telemetry.addData("State", tuner.getState());

            switch (tuner.getState()) {
                case IDLE:
                    telemetry.addData("Action", "Press A to start");
                    break;

                case INCREASING_KP:
                    telemetry.addData("Testing kP", "%.4f", tuner.getCurrentKP());
                    telemetry.addData("Zero crossings", tuner.getZeroCrossings());
                    telemetry.addData("Peak error", "%.1f", tuner.getPeakError());
                    telemetry.addData("Motor position", motor.getCurrentPosition());
                    break;

                case MEASURING:
                    telemetry.addData("Found Ku!", "%.4f", tuner.getUltimateGain());
                    telemetry.addData("Measuring period", "...");
                    break;

                case DONE:
                    double[] results = tuner.getResults();
                    telemetry.addLine("\n TUNING COMPLETE!");
                    telemetry.addData("kP", "%.6f", results[0]);
                    telemetry.addData("kI", "%.6f", results[1]);
                    telemetry.addData("kD", "%.6f", results[2]);
                    telemetry.addData("Ku (ultimate gain)", "%.6f", tuner.getUltimateGain());
                    telemetry.addData("Tu (period)", "%.4f sec", tuner.getUltimatePeriod());
                    telemetry.addLine("\nPress B to SAVE | X to TEST | Y to RESET");
                    break;

                case FAILED:
                    telemetry.addLine("\n TUNING FAILED");
                    telemetry.addData("Last kP tried", "%.4f", tuner.getCurrentKP());
                    telemetry.addLine("Try increasing kpMax or decreasing kpStep");
                    telemetry.addLine("Press Y to reset");
                    break;
            }
        }

        telemetry.update();
    }
}
