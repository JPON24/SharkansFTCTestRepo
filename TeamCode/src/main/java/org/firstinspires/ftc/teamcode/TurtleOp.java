package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.global.drivetrain.CoaxialSwerve;
import org.firstinspires.ftc.teamcode.global.util.gamepad.EnhancedGamepad;


@TeleOp
public class TurtleOp extends OpMode{

    CoaxialSwerve drive = new CoaxialSwerve();

    EnhancedGamepad g1 = null;
    EnhancedGamepad g2 = null;

    SparkFunOTOS otos;
    VirtualGoalShooter shooter;
    floatingIntake intake;
    AprilTagLimelight limelight;

    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;

    private boolean lastHoodUp = false;
    private boolean lastHoodDown = false;

    private boolean AtAutoFire = false;

    public double hoodAdjust = 0;

    private boolean isAutoAdjust = false;

    private boolean lastSwapAuto = false;

    // Manual shooter control
    private boolean manualMode = false;
    private double manualRPM = 3300;
    private double manualHood = 0.3;

    WorkingSwerve swerveDrive = new WorkingSwerve();

    @Override
    public void init() {

        g1 = new EnhancedGamepad(gamepad1);
        g2 = new EnhancedGamepad(gamepad2);
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.calibrateImu();
        otos.setAngularScalar(1);
        otos.setLinearScalar(1);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, -3.3105, -90));

//        otos.resetTracking();
//        otos.begin();
//
//        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        limelight = new AprilTagLimelight();
        limelight.init(hardwareMap, 0); // Uhhhh... Chat can we just use one pipeline??

        swerveDrive.init(hardwareMap);

//        drive.CoaxialSwerveConstants(1.0, 0.98, 0, 0);

//        drive.initFrontLeft(hardwareMap, "frontLeftServo", "frontLeftMotor", "frontLeftAnalog",
//                0.02, 0.003, 0.0, 0, 0); // 12.5 is the offset
//
//        // Front Right
//        drive.initFrontRight(hardwareMap, "frontRightServo", "frontRightMotor", "frontRightAnalog",
//                0.02, 0.003, 0.0, 0, 0);
//
//        // Back Left
//        drive.initBackLeft(hardwareMap, "backLeftServo", "backLeftMotor", "backLeftAnalog",
//                0.02, 0.003, 0.0, 0, 0);
//
//        // Back Right
//        drive.initBackRight(hardwareMap, "backRightServo", "backRightMotor", "backRightAnalog",
//                0.02, 0.003, 0.0, 0, 0);

        shooter = new VirtualGoalShooter();
        shooter.init(hardwareMap, otos);

        shooter.switchAlliance(true, false, false); // 2x2 alliance thingy for far and close + color. True no position sets it to 10 inches in front of you

        intake = new floatingIntake();
        intake.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
//
//        zephyr = new ZephyrSubsystem(hardwareMap, otos, limelight);
//        zephyr.setAlliance(true); // true for BLUE, false for RED
    }

    boolean outtaking = false;
    boolean lastOuttaking = false;
    double tempTgtRPM = 0;
    double tempTgtHood = 0;

    ElapsedTime hoodTimer = new ElapsedTime();
    boolean canSwap = false;

    boolean isAutoFire = false;
    boolean lastYButton = false;

    @Override
    public void loop() {

        // Sinex Controller

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

        // Y button toggles manual/auto mode
        if (gamepad2.y && !lastYButton) {
            manualMode = !manualMode;
        }
        lastYButton = gamepad2.y;

        // Manual RPM: dpad up/down (±100 per press)
        if (manualMode) {
            if (willIncrement) manualRPM += 100;
            if (willDecrement) manualRPM -= 100;
            manualRPM = Math.max(0, Math.min(6000, manualRPM));

            // Manual hood: bumpers (±0.05 per press)
            if (hoodUp && !lastHoodUp) manualHood += 0.05;
            if (hoodDown && !lastHoodDown) manualHood -= 0.05;
            manualHood = Math.max(0, Math.min(0.65, manualHood));
            lastHoodUp = hoodUp;
            lastHoodDown = hoodDown;
        }

        if (gamepad2.left_trigger > 0.3) {
            outtaking = false;
            intake.intake(true);
        } else if (gamepad2.right_trigger > 0.3) {
            intake.outtake(true);
            outtaking = true;
        } else {
            outtaking = false;
            intake.intake(false);
            intake.outtake(false);
            intake.outFront(false);
        }

        if (gamepad2.b) {
            shooter.spinUpShooter();

            if (shooter.isReadyToShoot()) {
                gamepad2.rumble(500);
            }
        } else {
            shooter.stopShooter();
            gamepad2.stopRumble();
        }

        if (manualMode) {
            // Manual mode: bypass VGS, set RPM and hood directly
            shooter.setShooterRPM(manualRPM);
            shooter.setManualHood(manualHood);
        } else {
            // Auto mode: let VGS handle everything
            shooter.update();
        }



//        g1.update();
//        g2.update();


        // Elias Controller


        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;;

//        drive.drive(leftStickX, leftStickY, rightStickX, 0);
//        drive.updateServos();

        swerveDrive.swerveDrive(leftStickY,leftStickX,rightStickX);


//        boolean plant = g1.lt.isDown();
//        boolean reset = g1.rt.isDown();


        if (gamepad1.right_trigger > 0.4)
        {
            swerveDrive.resetOTOSTracking();
        }
        telemetry.addData("P", shooter.getTeleP());
        telemetry.addData("D", shooter.getTeleDyeah());
        telemetry.addData("MODE", manualMode ? "MANUAL" : "AUTO");
        telemetry.addData("RPM (current/target)", "%.0f / %.0f", shooter.getCurrentRPM(),
                manualMode ? manualRPM : shooter.getTargetRPM());
        telemetry.addData("Hood", manualMode ? String.format("%.2f (manual)", manualHood) : String.format("%.2f (auto)", shooter.getHoodAngle()));
        telemetry.addData("Turret Output Angle", "%.1f°", shooter.outputAngle);
        telemetry.addData("Valid Shot", shooter.isValidShot());
        telemetry.addData("Distance", "%.1f in", shooter.getDistance());
        telemetry.addData("Turret Pos", "%.1f°", shooter.getTurretAngle());
        telemetry.addData("Turret Target", "%.1f°", shooter.getTargetTurretAngle());
        telemetry.addData("Turret Error", "%.1f°", shooter.getTurretError());

//        telemetry.addData("FL Voltage", drive.getFLVoltage());
//        telemetry.addData("FR Voltage", drive.getFRVoltage());
//        telemetry.addData("BL Voltage", drive.getBLVoltage());
//        telemetry.addData("BR Voltage", drive.getBRVoltage());
//
//        telemetry.addData("FL Angle", drive.frontLeftServo.getAngle());
//        telemetry.addData("FR Angle", drive.frontRightServo.getAngle());
//        telemetry.addData("BL Angle", drive.backLeftServo.getAngle());
//        telemetry.addData("BR Angle", drive.backRightServo.getAngle());
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

//        telemetry.addData("flSpeed: ", swerve.flSpeed);
//        telemetry.addData("frSpeed: ", swerve.frSpeed);
//        telemetry.addData("blSpeed: ", swerve.blSpeed);
//        telemetry.addData("brSpeed: ", swerve.brSpeed);
//        telemetry.addData("is decelerating: ", swerve.decelerating);
//
//        telemetry.addData("x cmd: ", swerve.xCmdVal);
//        telemetry.addData("y cmd: ", swerve.yCmdVal);
//        telemetry.addData("r cmd: ", swerve.rCmdVal);

        telemetry.addData("otos x: ", otos.getPosition().x);
        telemetry.addData("otos y: ", otos.getPosition().y);
        telemetry.addData("otos h: ", otos.getPosition().h);

        double xval = shooter.BLUE_BASKET_FAR.x - otos.getPosition().y;
        double yval = shooter.BLUE_BASKET_FAR.y - (-otos.getPosition().x);

        telemetry.addData("mag: ", Math.hypot(xval, yval));

        telemetry.addData("FL OFFSET", swerveDrive.getFLRaw());
        telemetry.addData("FR OFFSET", swerveDrive.getFRRaw());
        telemetry.addData("BL OFFSET", swerveDrive.getBLRaw());
        telemetry.addData("BR OFFSET", swerveDrive.getBRRaw());

        telemetry.addData("Max Error", "%.2f°", swerveDrive.getMaxError());
        telemetry.addData("FL Error", "%.2f°", swerveDrive.getFLError());
        telemetry.addData("FR Error", "%.2f°", swerveDrive.getFRError());
        telemetry.addData("BL Error", "%.2f°", swerveDrive.getBLError());
        telemetry.addData("BR Error", "%.2f°", swerveDrive.getBRError());

//")
        //        telemetry.addData("heading: ", swerve.heading());
        telemetry.update();
    }
}
