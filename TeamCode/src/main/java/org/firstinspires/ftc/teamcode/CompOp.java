package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Competition TeleOp")
public class CompOp extends OpMode {

    private SwerveSubsystem swerve;
    private ShooterSubsystem shooter;
    private floatingIntake intake;

    private SparkFunOTOS otos;

    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;

    @Override
    public void init() {
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(new SparkFunOTOS.Pose2D(0, -3.74016, 0));
        otos.calibrateImu();

        // Initialize subsystems
        swerve = new SwerveSubsystem();
        swerve.init(hardwareMap);

        shooter = new ShooterSubsystem();
        shooter.init(hardwareMap, otos);

        intake = new floatingIntake();
        intake.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        swerve.drive(leftStickY, leftStickX, rightStickX);

        boolean shootButtonPressed = gamepad1.a;
        boolean hardShotPressed = gamepad1.b;
        boolean rightBumperPressed = gamepad1.right_bumper;
        boolean leftBumperPressed = gamepad1.left_bumper;

        boolean willIncrement = (rightBumperPressed && !lastRightBumperState);
        boolean willDecrement = (leftBumperPressed && !lastLeftBumperState);
        
        if (willIncrement) {
            if (shooter.getTargetRPM() + 100 < 6000) {
                shooter.setTargetRPM(shooter.getTargetRPM() + 100);
            }
        } else if (willDecrement) {
            if (shooter.getTargetRPM() - 100 >= 0) {
                shooter.setTargetRPM(shooter.getTargetRPM() - 100);
            }
        }

        lastRightBumperState = rightBumperPressed;
        lastLeftBumperState = leftBumperPressed;

        shooter.setTargetRPM(shooter.getTargetRPM());

        shooter.update(shootButtonPressed, hardShotPressed);

        // Press Y to calibrate and switch to predictive mode
        if (gamepad1.y) {
            shooter.requestCalibration();
        }
        
        shooter.trackTargetHybrid();

        if (gamepad1.dpad_left) {
            intake.intake(true);
        } else if (gamepad1.dpad_right) {
            intake.outtake(true);
        } else {
            intake.intake(false);
            intake.outtake(false);
        }

        telemetry.addData("rpm", shooter.getCurrentRPM());
        telemetry.addData("=== SWERVE ===", "");
        telemetry.addData("Wheels (FL/FR/BL/BR)", "%.0f° %.0f° %.0f° %.0f°",
                swerve.getFLAngle(), swerve.getFRAngle(),
                swerve.getBLAngle(), swerve.getBRAngle());
        telemetry.addData("", "");
        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("RPM (current/target)", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Turret", "%.1f° (%d ticks)", shooter.getTurretAngle(), shooter.getTurretTicks());
        telemetry.addData("Mode", shooter.isCalibrationMode() ? "REACTIVE" : "PREDICTIVE");
        telemetry.addData("Calibrated", shooter.isCalibrated() ? "YES" : "NO");
        telemetry.addData("Saved Offset", "%.1f°", shooter.getSavedOffset());
        
        if (!shooter.isCalibrationMode()) {
            // Show predictive mode details
            telemetry.addData("LL IMU Yaw", "%.1f°", shooter.getLimelightYaw());
            telemetry.addData("Target Angle", "%.1f°", shooter.getLastTargetAngle());
            telemetry.addData("Turret Current", "%.1f°", shooter.getTurretAngle());
            telemetry.addData("Error", "%.1f°", shooter.getLastTargetAngle() - shooter.getTurretAngle());
        } else {
            telemetry.addData("DEBUG TX Error", "%.2f°", shooter.getLastTX());
            telemetry.addData("DEBUG Deadband", "%.1f°", shooter.getDeadband());
        }
        telemetry.update();
    }
}
