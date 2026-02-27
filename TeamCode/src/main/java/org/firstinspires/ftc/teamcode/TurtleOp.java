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

    @Override
    public void loop() {

//        g1.update();
//        g2.update();

//        double leftStickX = g1.left_stick_x();
//        double leftStickY = g1.left_stick_y();
//        double rightStickX = -g1.right_stick_x();;


        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;;

//        drive.drive(leftStickX, leftStickY, rightStickX, 0);
//        drive.updateServos();

        swerveDrive.swerveDrive(leftStickY,leftStickX,rightStickX);

        shooter.update();

//        boolean plant = g1.lt.isDown();
//        boolean reset = g1.rt.isDown();


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

        if (upDpadPressed) {
            shooter.spinUpShooter();

            if (shooter.isReadyToShoot()) {
                gamepad2.rumble(500);
            }
        } else {
            shooter.stopShooter();
            gamepad2.stopRumble();
        }

        if (gamepad1.left_trigger > 0.3) {
            outtaking = false;
            intake.intake(true);
        } else if (gamepad2.right_trigger > 0.3) {
            intake.outtake(true);
            outtaking = true;
        } else if (reverseIntake) {
            intake.outFront(true);
        }
        else {
            outtaking = false;
            intake.intake(false);
            intake.outtake(false);
            intake.outFront(false);
        }

        if (gamepad1.right_trigger > 0.4)
        {
            swerveDrive.resetOTOSTracking();
        }

        telemetry.addData("RPM (current/target)", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("SS angle output", shooter.getHoodAngle());

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

        telemetry.addData("otos x: ", otos.getPosition().x * 1.6);
        telemetry.addData("otos y: ", otos.getPosition().y * 1.6);
        telemetry.addData("otos h: ", otos.getPosition().h);
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
