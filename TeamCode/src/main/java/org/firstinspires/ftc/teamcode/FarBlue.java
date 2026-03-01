package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class FarBlue extends LinearOpMode//lemme in pls
{
    SharkDrive shark = new SharkDrive();
    MoveCommand moveCmd = new MoveCommand(); //lemme in 2


    ShooterSubsystem shooter = new ShooterSubsystem();
    WorkingSwerve swerve = new WorkingSwerve();
    floatingIntake intake = new floatingIntake();
    SparkFunOTOS otos;

    private final double autonSpeed = 1;

    @Override
    public void runOpMode()
    {
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.calibrateImu();
        otos.setAngularScalar(1);
        otos.setLinearScalar(1);

        otos.resetTracking();
        otos.begin();

        otos.setOffset(new SparkFunOTOS.Pose2D(0, -3.3105, -90));

        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        shark.init(hardwareMap, true, otos);
        moveCmd.init(hardwareMap, true, shark);

        shooter.initSystem(hardwareMap, otos, 0);
        swerve.init(hardwareMap);
        intake.init(hardwareMap);

        swerve.swerveDrive(0,0,0);

        waitForStart();
        while (opModeIsActive())
        {
//            telemetry.addData("otos x:", otos.getPosition().y);
//            telemetry.addData("otos y:", -otos.getPosition().x);
//            telemetry.addData("otos h:", otos.getPosition().h);
//            telemetry.update();

//            moveCmd.MoveToPosition(autonSpeed, -20, 0, 0, 2, 2, 0, 0, 0, false, false);

            ShootInit(0);

            SpikeMarkOne(0);
            Shoot(0);

            HumanPlayerIntake(0);
            Shoot(0);

            HumanPlayerIntake(0);
            Shoot(0);

            HumanPlayerIntake(0);
            Shoot(0);

            HumanPlayerIntake(0);
            Shoot(0);

            HumanPlayerIntake(0);
            Shoot(0);
            break;
        }
    }

    /*



     */

    private void ShootInit(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 2, 0, 0, 2, 2, -11, 0, 5000, false, false);
        moveCmd.MoveToPosition(0, 2, 0, 0, 2, 4, -11, 0, 5000, false, false);
        sleep(1000);
        moveCmd.MoveToPosition(0, 2, 0, 0, 2, 4, -11, 0, 5000, false, true);
        sleep(500);
    }

    private void Shoot(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 2, 0, 0, 2, 2, -11, 0, 5000, true, false);
        moveCmd.MoveToPosition(0, 2, 0, 0, 2, 4, -11, 0, 5000, false, true);
        sleep(500);
    }

    private void SpikeMarkOne(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 28, 0, 0, 2, 2, -11, 0, 5000, true, false);
        moveCmd.MoveToPosition(autonSpeed, 28, 24,0, 2, 2, -11, 0, 5000, true, false);
        moveCmd.MoveToPosition(autonSpeed, 28, 40,0, 2, 2, -11, 0, 5000, true, false);
        moveCmd.MoveToPosition(autonSpeed, 28, 50,0, 2, 2, -11, 0, 5000, true, false);
        moveCmd.MoveToPosition(0, 28, 50,0, 2, 4, -11, 0, 5000, true, false);
        sleep(500);
    }

    private void HumanPlayerIntake(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 2, 52, 0, 2, 2, -11, 0, 5000, true, false);
        moveCmd.MoveToPosition(0, 2, 52, 0, 2, 4, -11, 0, 5000, true, false);
        sleep(500);
    }
}