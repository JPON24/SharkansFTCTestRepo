package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SwerveSubsystem;

@TeleOp(name = "Competition TeleOp BLUE")
public class CompOp extends OpMode {

    private SwerveSubsystem swerve;
    private ShooterSubsystem shooter;
    private floatingIntake intake;

    private SparkFunOTOS otos;

    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;

    private boolean lastHoodUp = false;
    private boolean lastHoodDown = false;

    public double hoodAdjust = 0;

    private boolean isAutoAdjust = false;

    private boolean lastSwapAuto = false;

    @Override
    public void init() {
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(new SparkFunOTOS.Pose2D(0, -3.74016, 0));
        otos.calibrateImu();

        // Initialize subsystems
        swerve = new SwerveSubsystem();
        swerve.init(hardwareMap, otos);  // Pass OTOS for field-centric

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

        boolean rightBumperPressed = gamepad1.right_bumper;
        boolean leftBumperPressed = gamepad1.left_bumper;

        boolean hoodUp = gamepad1.dpad_right;
        boolean hoodDown = gamepad1.dpad_left;

        boolean willIncrement = (rightBumperPressed && !lastRightBumperState);
        boolean willDecrement = (leftBumperPressed && !lastLeftBumperState);

        boolean swapToAuto = gamepad1.a;

        if (swapToAuto && !lastSwapAuto) {
            isAutoAdjust = !isAutoAdjust;
        }

        lastSwapAuto = swapToAuto;

        if (isAutoAdjust)
        {
            shooter.update();
        }
        else
        {
            shooter.setHoodPosition(hoodAdjust);

            if (willIncrement) {
                if (shooter.getTargetRPM() + 100 < 6000) {
                    shooter.setTargetRPM(shooter.getTargetRPM() + 100);
                    shooter.BangBang();
                }
            } else if (willDecrement) {
                if (shooter.getTargetRPM() - 100 >= 0) {
                    shooter.setTargetRPM(shooter.getTargetRPM() - 100);
                }
            }
        }

        if (hoodUp && !lastHoodUp && hoodAdjust + 0.05 <= 0.65) {
            hoodAdjust = hoodAdjust + 0.05;
        } else if (hoodDown && !lastHoodDown && hoodAdjust - 0.05 >= 0){
            hoodAdjust -= 0.05;
        }

        telemetry.addData("hood position", hoodAdjust);

//        if (shooter.getCurrentRPM() > 0) {
////            shooter.setLEDLight();
        shooter.BangBang();
//        }

        lastRightBumperState = rightBumperPressed;
        lastLeftBumperState = leftBumperPressed;

        lastHoodUp = hoodUp;
        lastHoodDown = hoodDown;

//        shooter.setTargetRPM(shooter.getTargetRPM());


        // Press Y to calibrate and switch to predictive mode
//        if (gamepad1.y) {
//            shooter.requestCalibration();
//        }

        if (gamepad1.right_trigger > 0.5)
        {
            shooter.decideManualOrTxBLUE(-1);
        }
        else if (gamepad1.left_trigger > 0.5)
        {
            shooter.decideManualOrTxBLUE(1);
        }
        else {
            shooter.decideManualOrTxBLUE(0);
        }
//        shooter.trackTargetHybrid();

        if (gamepad1.x) {
            intake.intake(true);
        } else if (gamepad1.y) {
            intake.outtake(true);
        } else {
            intake.intake(false);
            intake.outtake(false);
        }

//        telemetry.addData("rpm", shooter.getCurrentRPM());
//        telemetry.addData("=== OTOS POSITION ===", "");
//        telemetry.addData("X", "%.2f", otos.getPosition().x);
//        telemetry.addData("Y", "%.2f", otos.getPosition().y);
//        telemetry.addData("Heading", "%.1f°", Math.toDegrees(otos.getPosition().h));
//        telemetry.addData("=== SWERVE ===", "");
//        telemetry.addData("Wheels (FL/FR/BL/BR)", "%.0f° %.0f° %.0f° %.0f°",
//                swerve.getFLAngle(), swerve.getFRAngle(),
//                swerve.getBLAngle(), swerve.getBRAngle());
//        telemetry.addData("RawServoAngle, (FL/FR/BL/BR", "%.0f° %.0f° %.0f° %.0f°",
//                swerve.getFLRawAngle(), swerve.getFRRawAngle(),
//                swerve.getBLRawAngle(), swerve.getBRRawAngle());
//        telemetry.addData("", "");
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

        telemetry.addData("FL tgt: ", swerve.angleFL);
        telemetry.addData("FR tgt: ", swerve.angleFR);
        telemetry.addData("BL tgt: ", swerve.angleRL);
        telemetry.addData("BR tgt: ", swerve.angleRR);

        telemetry.addData("FL tgt POS: ", swerve.lastTargetFL);
        telemetry.addData("FR tgt POS: ", swerve.lastTargetFR);
        telemetry.addData("BL tgt POS: ", swerve.lastTargetRL);
        telemetry.addData("BR tgt POS: ", swerve.lastTargetRR);

        telemetry.addData("Distance", shooter.getDistance());
        telemetry.update();
    }
}
