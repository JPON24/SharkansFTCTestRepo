package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class CloseBlue extends LinearOpMode//lemme in pls
{
    SharkDrive shark = new SharkDrive();
    MoveCommand moveCmd = new MoveCommand(); //lemme in 2


    ShooterSubsystem shooter = new ShooterSubsystem();
    WorkingSwerve swerve = new WorkingSwerve();
    floatingIntake intake = new floatingIntake();
    SparkFunOTOS otos;

    private final double autonSpeed = 0.7;

    @Override
    public void runOpMode()
    {
        shark.init(hardwareMap, true);
        moveCmd.init(hardwareMap, true);

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(new SparkFunOTOS.Pose2D(0, -3.74016, 0));
        otos.setPosition(new SparkFunOTOS.Pose2D(10, 10, -45)); //set later
        otos.calibrateImu();

        shooter.initSystem(hardwareMap, otos, 0);
        swerve.init(hardwareMap);
        intake.init(hardwareMap);

        swerve.swerveDrive(0,0,0);

        waitForStart();
        while (opModeIsActive())
        {
            Shoot(0);

            SpikeMarkTwo(0);
            Shoot(0);

            GateIntake(0);
            Shoot(0);

            GateIntake(0);
            Shoot(0);

            SpikeMarkOne(0);
            ShootLeave(0);

//            // shoot preload
//            shooter.setTargetRPM(3300);
//            shooter.setHoodPosition(0.45);
//            while (otos.getPosition().y > -51) {
//                telemetry.addData("what the flip brotha: ", 1);
//                telemetry.update();
//                swerve.robotCentric(-1 * autonSpeed, 0, 0);
//            }
//            swerve.stop();
//            sleep(500);
//            intake.outtake(true);
//            sleep(500);
//            intake.outtake(false);
//
//            // second spike mark
//            while (otos.getPosition().h < 44)
//            {
//                swerve.robotCentric(0,0, 1 * autonSpeed);
//            }
//            swerve.stop();
//
//            intake.intake(true);
//
//            while (otos.getPosition().x > -52.5)
//            {
//                swerve.robotCentric(1*autonSpeed,0,0);
//            }
//            swerve.stop();
//            intake.intake(false);
//
//            // shoot
//            while (otos.getPosition().x < -9)
//            {
//                shooter.turnToAngle(-44, false);
//                swerve.drive(1 * autonSpeed, 0.34 * autonSpeed, 0);
//            }
//            swerve.stop();
//            sleep(500);
//            intake.outtake(true);
//            sleep(500);
//            intake.outtake(false);
//
//            // gate intake
//            while (otos.getPosition().h > 10)
//            {
//                shooter.turnToAngle(-10,false);
//                swerve.drive(0,0,-0.5*autonSpeed);
//            }
//            swerve.stop();
//
//            while (otos.getPosition().x > -49.9)
//            {
//                swerve.drive(-0.34, 1, 0);
//            }
//            swerve.stop();
//
//            intake.intake(true);
//            sleep(1500);
//            intake.intake(false);
//
//            // shoot
//            while (otos.getPosition().x < -9)
//            {
//                swerve.drive(1 * autonSpeed, 0.34 * autonSpeed, 0);
//            }
//            swerve.stop();
//            sleep(500);
//            intake.outtake(true);
//            sleep(500);
//            intake.outtake(false);
//
//            // gate intake
//            while (otos.getPosition().x > -49.9)
//            {
//                swerve.drive(-0.34, 1, 0);
//            }
//            swerve.stop();
//
//            intake.intake(true);
//            sleep(1500);
//            intake.intake(false);
//
//            // shoot
//            while (otos.getPosition().x < -9)
//            {
//                swerve.drive(1 * autonSpeed, 0.34 * autonSpeed, 0);
//            }
//            swerve.stop();
//            sleep(500);
//            intake.outtake(true);
//            sleep(500);
//            intake.outtake(false);
//            swerve.robotCentric(0,1,0);
//            sleep(1000);
//            swerve.robotCentric(0,0,0);

            telemetry.addData("X", shark.GetX());
            telemetry.addData("Y", shark.GetY());
            telemetry.addData("Heading", shark.GetH());
            telemetry.addData("X Error",shark.GetErrorX());
            telemetry.addData("Y Error", shark.GetErrorY());
            telemetry.addData("Heading Error", shark.GetErrorH());


            break;



        }


//        telemetry.addData("x position: ", otos.getPosition().x);
//        telemetry.addData("y position: ", otos.getPosition().y);
//
//        telemetry.addData("Dih error", shark.previousDihError);
//        telemetry.update();
    }

    /*
shoot - -9, -49 0

intake 2 - -31, -60, 44

intake spike mark -52.5, -34.3

GATE INTAKE - -49.9, -35, 10

     */

    // offset if otos drifts heavily
    private void Shoot(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -2, -42, 0, 1, 1, 0, 0.45, 3300, false, false);
        moveCmd.MoveToPosition(0, -2, -42, 0, 1, 4, 0, 0.45, 3300, false, false);
        sleep(500);
        moveCmd.MoveToPosition(autonSpeed, -2, -42, 0, 1, 4, 0, 0.45, 3300, false, true);
        sleep(500);
    }

    private void ShootLeave(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 2, 42, 30, 1, 2, 60, 0.45, 3300, false, false);
        moveCmd.MoveToPosition(autonSpeed, 2, 42, 30, 1, 4, 60, 0.45, 3300, false, true);
        sleep(500);
        moveCmd.MoveToPosition(autonSpeed, 6, 44, 30, 1, 2, 60, 0.45, 3300, false, false);
    }

    private void SpikeMarkOne(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -30, -37, 30, 1, 2, 60, 0.45, 3300, true, false);
        moveCmd.MoveToPosition(autonSpeed, -38.3, -17.7, 30, 1, 2, 60, 0.45, 3300, true, false);
    }

    private void SpikeMarkTwo(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -54, -46, 30, 1, 2, 60, 0.45, 3300, true, false);
        moveCmd.MoveToPosition(autonSpeed, -60.53, -26.15, 30, 1, 2, 60, 0.45, 3300, true, false);
    }

    private final int gateIntakeTimingMs = 2000;
    private void GateIntake(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -56.25, -20, 5, 1, 3, 60, 0.45, 3300, true, true);
        sleep(gateIntakeTimingMs); // tune this to figure out gate intake timing
    }
}
