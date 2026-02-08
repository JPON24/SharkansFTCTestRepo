package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        shooter.initSystem(hardwareMap, otos, 0);

        intake = new floatingIntake();
        intake.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        swerve.drive(0, 0, 0);
//
//        zephyr = new ZephyrSubsystem(hardwareMap, otos, limelight);
//        zephyr.setAlliance(true); // true for BLUE, false for RED
    }

    boolean outtaking = false;
    boolean lastOuttaking = false;
    double tempTgtRPM = 0;
    double tempTgtHood = 0;

    /*
    new shooter positions

     */

    ElapsedTime hoodTimer = new ElapsedTime();
    boolean canSwap = false;

    @Override
    public void loop() {

//        zephyr.update();

        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = -gamepad1.right_stick_x;


        double plant = gamepad1.left_trigger;
        double reset = gamepad1.right_trigger;

        if (plant > 0.4)
        {
            swerve.plant();
        }
        else
        {
            swerve.drive(leftStickY, leftStickX, rightStickX);
        }

        if (reset > 0.4)
        {
            swerve.resetIMU();
        }

        boolean upDpadPressed = gamepad2.dpad_up;
        boolean downDpadPressed = gamepad2.dpad_down;

        boolean hoodUp = gamepad2.right_bumper;
        boolean hoodDown = gamepad2.left_bumper;

        boolean willIncrement = (upDpadPressed && !lastRightBumperState);
        boolean willDecrement = (downDpadPressed && !lastLeftBumperState);

        boolean setShooterRPMHigh = gamepad2.y;
        boolean setShooterRPMLow = gamepad2.x;
        boolean swapToAuto = gamepad2.b;
        boolean reverseIntake = gamepad2.a;

        boolean turretPos = gamepad2.dpad_left;
        boolean turretNeg = gamepad2.dpad_right;

        if (swapToAuto && !lastSwapAuto) {
            isAutoAdjust = !isAutoAdjust;
        }

        lastSwapAuto = swapToAuto;

        if (isAutoAdjust && !outtaking)
        {
            shooter.update();

            if (shooter.IsAtTgtRPM() && shooter.getTargetRPM() != 0)
            {
                gamepad2.rumble(500);
            }
        }
        else
        {
            shooter.setHoodPosition(hoodAdjust);

            if (willIncrement) {
                if (shooter.getTargetRPM() + 50 < 6000) {
                    shooter.setTargetRPM(shooter.getTargetRPM() + 50);
                    shooter.BangBang();
                }
            } else if (willDecrement) {
                if (shooter.getTargetRPM() - 50 >= 0) {
                    shooter.setTargetRPM(shooter.getTargetRPM() - 50);
                }
            }
        }

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
        telemetry.addData("turret position: ", shooter.getTurretTicks());
        telemetry.addData("turret mode: ", shooter.getTurretState());

        if (turretPos)
        {
            shooter.txTracking(1);
        }
        else if (turretNeg)
        {
            shooter.txTracking(-1);
        }
        else
        {
            shooter.txTracking(0);
        }

        shooter.BangBang();

        lastRightBumperState = upDpadPressed;
        lastLeftBumperState = downDpadPressed;

        lastHoodUp = hoodUp;
        lastHoodDown = hoodDown;

        if (hoodTimer.seconds() > 0.1)
        {
            shooter.setHoodPosition(tempTgtHood);
        }

        if (gamepad2.left_trigger > 0.3) {
            outtaking = false;
            intake.intake(true);
        } else if (gamepad2.right_trigger > 0.3) {
            intake.outtake(true);
            outtaking = true;
            if (isAutoAdjust && !lastOuttaking)
            {
                tempTgtHood = shooter.currentHood;
                tempTgtRPM = shooter.currentRpm;
                hoodTimer.reset();

                if (shooter.IsAtTgtRPM() && shooter.getTargetRPM() != 0)
                {
                    gamepad2.rumble(500);
                }

                shooter.setTargetRPM(tempTgtRPM);
                shooter.setHoodPosition(Math.min(0, tempTgtHood - 0.1));
            }
        } else if (reverseIntake) {
            intake.outFront(true);
        }
        else {
            outtaking = false;
            intake.intake(false);
            intake.outtake(false);
            intake.outFront(false);
        }

        lastOuttaking = outtaking;

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
        telemetry.addData("RPM (current/target)", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
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
        telemetry.addData("is decelerating: ", swerve.decelerating);

        telemetry.addData("x cmd: ", swerve.xCmdVal);
        telemetry.addData("y cmd: ", swerve.yCmdVal);
        telemetry.addData("r cmd: ", swerve.rCmdVal);

        telemetry.addData("otos x: ", otos.getPosition().x);
        telemetry.addData("otos y: ", otos.getPosition().y);
        telemetry.addData("otos h: ", otos.getPosition().h);
//        telemetry.addData("heading: ", swerve.heading());
        telemetry.addData("Tx: ", shooter.getTx());
        telemetry.addData("Distance", shooter.getDistance());
        telemetry.update();
    }
}
