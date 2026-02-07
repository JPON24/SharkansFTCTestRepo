package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SwerveSubsystem;

@TeleOp(name = "Competition TeleOp BLUE")
public class CompOp extends OpMode {

    private ZephyrSubsystem zephyr;
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
        shooter.init(hardwareMap, otos, 20);

        intake = new floatingIntake();
        intake.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        swerve.drive(0, 0, 0);
//
//        zephyr = new ZephyrSubsystem(hardwareMap, otos, limelight);
//        zephyr.setAlliance(true); // true for BLUE, false for RED
    }

    @Override
    public void loop() {

//        zephyr.update();

        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = -gamepad1.right_stick_x;

        swerve.drive(leftStickY, leftStickX, rightStickX);

        boolean upDpadPressed = gamepad2.dpad_up;
        boolean downDpadPressed = gamepad2.dpad_down;

        boolean hoodUp = gamepad2.right_bumper;
        boolean hoodDown = gamepad2.left_bumper;

        boolean willIncrement = (upDpadPressed && !lastRightBumperState);
        boolean willDecrement = (downDpadPressed && !lastLeftBumperState);

        boolean setShooterRPMHigh = gamepad2.y;
        boolean setShooterRPMLow = gamepad2.a;
        boolean swapToAuto = gamepad2.b;

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

        shooter.txTracking();

        if (setShooterRPMHigh)
        {
            shooter.setTargetRPM(3300);
        }

        if (setShooterRPMLow)
        {
            shooter.setTargetRPM(0);
        }

        if (hoodUp && !lastHoodUp && hoodAdjust + 0.05 <= 0.65) {
            hoodAdjust = hoodAdjust + 0.05;
        } else if (hoodDown && !lastHoodDown && hoodAdjust - 0.05 >= 0){
            hoodAdjust -= 0.05;
        }

        telemetry.addData("hood position", hoodAdjust);

        shooter.BangBang();

        lastRightBumperState = upDpadPressed;
        lastLeftBumperState = downDpadPressed;

        lastHoodUp = hoodUp;
        lastHoodDown = hoodDown;

        if (gamepad2.left_trigger > 0.3) {
            intake.intake(true);
        } else if (gamepad2.right_trigger > 0.3) {
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
//        telemetry.addData("=== SHOOTER ===", "");
//        telemetry.addData("RPM (current/target)", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
//        telemetry.addData("Turret", "%.1f° (%d ticks)", shooter.getTurretAngle(), shooter.getTurretTicks());
//        telemetry.addData("Mode", shooter.isCalibrationMode() ? "REACTIVE" : "PREDICTIVE");
//        telemetry.addData("Calibrated", shooter.isCalibrated() ? "YES" : "NO");
//        telemetry.addData("Saved Offset", "%.1f°", shooter.getSavedOffset());
//
//        if (!shooter.isCalibrationMode()) {
//            // Show predictive mode details
//            telemetry.addData("LL IMU Yaw", "%.1f°", shooter.getLimelightYaw());
//            telemetry.addData("Target Angle", "%.1f°", shooter.getLastTargetAngle());
//            telemetry.addData("Turret Current", "%.1f°", shooter.getTurretAngle());
//            telemetry.addData("Error", "%.1f°", shooter.getLastTargetAngle() - shooter.getTurretAngle());
//        } else {
//            telemetry.addData("DEBUG TX Error", "%.2f°", shooter.getLastTX());
//            telemetry.addData("DEBUG Deadband", "%.1f°", shooter.getDeadband());
//        }
//
//        telemetry.addData("FL tgt: ", swerve.angleFL);
//        telemetry.addData("FR tgt: ", swerve.angleFR);
//        telemetry.addData("BL tgt: ", swerve.angleRL);
//        telemetry.addData("BR tgt: ", swerve.angleRR);
//
//        telemetry.addData("FL tgt POS: ", swerve.lastTargetFL);
//        telemetry.addData("FR tgt POS: ", swerve.lastTargetFR);
//        telemetry.addData("BL tgt POS: ", swerve.lastTargetRL);
//        telemetry.addData("BR tgt POS: ", swerve.lastTargetRR);

        telemetry.addData("flSpeed: ", swerve.flSpeed);
        telemetry.addData("frSpeed: ", swerve.frSpeed);
        telemetry.addData("blSpeed: ", swerve.blSpeed);
        telemetry.addData("brSpeed: ", swerve.brSpeed);
        telemetry.addData("shouldDecelerate: " , swerve.shouldDecelerate);
        telemetry.addData("is decelerating: ", swerve.decelerating);
        telemetry.addData("can decelerate: ", swerve.canDecelerate);

        telemetry.addData("x cmd: ", swerve.xCmdVal);
        telemetry.addData("y cmd: ", swerve.yCmdVal);
        telemetry.addData("r cmd: ", swerve.rCmdVal);

        telemetry.addData("heading: ", swerve.heading());
        telemetry.addData("Tx: ", shooter.getTx());
        telemetry.addData("Distance", shooter.getDistance());
        telemetry.update();
    }
}
