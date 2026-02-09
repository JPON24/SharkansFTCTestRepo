package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "BlueLeave")
public class CloseBlueLeave extends LinearOpMode {

    private final double driveSpeed = 0.811;
    private final double shooterRpm = 3300.0;
    private final double countsPerWheelRev = 28.0;
    private final double turnPower = 0.5;
    private final int rotateDurationMs = 600;
    private final int forwardDurationMs = 1000;

    private SwerveSubsystem swerve = new SwerveSubsystem();

    private DcMotorEx rightShooter;
    private Servo leftHood;
    private Servo rightHood;
    private DcMotorSimple transferMotor;
    private DcMotorSimple intakeMotor;





    @Override
    public void runOpMode() {

        swerve.init(hardwareMap, null);

        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setDirection(DcMotorEx.Direction.FORWARD);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(100, 1, 0, 0));

        leftHood = hardwareMap.get(Servo.class, "leftHood");
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        leftHood.setDirection(Servo.Direction.REVERSE);


        transferMotor = hardwareMap.get(DcMotorSimple.class, "transferMotor");
        intakeMotor = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        waitForStart();
        if (isStopRequested()) {
            return;
        }



        swerve.robotCentric(0.1,0,0);
        sleep(1000);
        // Move backward
        swerve.robotCentric(0.4, 0, 0);
        sleep(2000); // init 4000
        swerve.stop();

        // Rotate CCW
//        swerve.robotCentric(-driveSpeed, 0, 0);
//        sleep(2000); // init 4000
//        swerve.stop();
//
//        swerve.robotCentric(0, 0, turnPower);
//        sleep(1200);
//        swerve.stop();
//
//        // Move forward
//        swerve.robotCentric(driveSpeed, 0, 0);
//        transferMotor.setPower(-0.16);
//        intakeMotor.setPower(1.0);
//        sleep(3000);
//        transferMotor.setPower(0);
//        intakeMotor.setPower(0);
//        swerve.stop();
//
//        swerve.robotCentric(-driveSpeed, 0, 0);
//        sleep(3500);
//        swerve.stop();
//
//        swerve.robotCentric(0, 0, -turnPower);
//        sleep(6000);
//        swerve.stop();
//
//        // Run transfer to shoot
//        transferMotor.setPower(-1.0);
//        intakeMotor.setPower(1.0);
//        sleep(5000);
//        transferMotor.setPower(0);
//        intakeMotor.setPower(0);
//        rightShooter.setVelocity(0);

    }
}