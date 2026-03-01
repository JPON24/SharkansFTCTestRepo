package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class CloseBlue extends LinearOpMode//lemme in pls
{
    SharkDrive shark = new SharkDrive();
    MoveCommand moveCmd = new MoveCommand(); //lemme in 2

    ElapsedTime timer = new ElapsedTime();


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
            Shoot(0);

            SpikeMarkTwo(0);
            Shoot(-1);
//
//            GateIntake(0, 0);
//            Shoot(0);
////
//            GateIntake(+1, 0.5);
//            Shoot(0);
//
            SpikeMarkOne(0);
            Shoot(-2);
            break;
        }
    }

    /*
    shoot = 25.5, -30, 0
    spike two prep = 16, -64, 0
    spike two = -12, -64, 0
    gate intake = -8, -61, -27
    spike one prep = 18.5, -41, 0
    spike one = -5, -41, 0

     */

    // offset if otos drifts heavily
    private void Shoot(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -30 + offset, -25.5 + offset, 0, 2, 2, -40, 0.45, 3200, false, false); // might have to swap intaking to true
        // do we need spin up time?
        timer.reset();
        while (timer.seconds() < 1)
        {
            moveCmd.MoveToPosition(0, -30 + offset, -25.5 + offset, 0, 2, 4, -40, 0.45, 3200, false, false);
        }

        moveCmd.MoveToPosition(autonSpeed, -30 + offset, -25.5 + offset, 0, 2, 4, -40, 0.45, 3200, false, true);
        sleep(500);
    }

    private void SpikeMarkOne(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -41, -16, 0, 2, 2, -40, 0.45, 3300, true, false);
//        moveCmd.MoveToPosition(0, -1.5, -40, 0, 2, 4, 0, 0.45, 3300, false, false);

//        sleep(1000);
        moveCmd.MoveToPosition(autonSpeed, -41, 6,0, 2, 2, -40, 0.45, 3300, true, false);
        moveCmd.MoveToPosition(0, -41, 6,0, 2, 4, -40, 0.45, 3300, true, false);
        sleep(500);

    }

    private void SpikeMarkTwo(double offset)
    {
        // either -63 or -65
        moveCmd.MoveToPosition(autonSpeed, -63, -20, 0, 2, 2, -40, 0.45, 3300, true, false);
//        moveCmd.MoveToPosition(0, -46, -24, 0, 2, 4, 0, 0.45, 3300, false, false);

//        sleep(1000);
        moveCmd.MoveToPosition(autonSpeed, -63, 6, 0, 2, 2, -40, 0.45, 3300, true, false);
        moveCmd.MoveToPosition(autonSpeed, -63, 16, 0, 2, 2, -40, 0.45, 3300, true, false);
        moveCmd.MoveToPosition(0, -63, 16, 0, 2, 4, -40, 0.45, 3300, true, false);
        sleep(500);

        moveCmd.MoveToPosition(autonSpeed, -63, -20, 0, 2, 2, -40, 0.45, 3300, false, false);

    }

    // 3 inch drift on gate intake
    private final int gateIntakeTimingMs = 2000;
    private void GateIntake(double offset, double y)
    {
        // -43.5, -40
        // rotate 17
        moveCmd.MoveToPosition(autonSpeed, -70 + offset, -10, -40, 2, 2, -40, 0.45, 3300, true, false);
        moveCmd.MoveToPosition(autonSpeed, -70 + offset, 11.5 + y, -40, 2, 2, -40, 0.45, 3300, true, false);
        moveCmd.MoveToPosition(0, -70 + offset, 11.5 + y, -40, 2, 4, -40, 0.45, 3300, true, false);
        sleep(gateIntakeTimingMs); // tune this to figure out gate intake timing
        moveCmd.MoveToPosition(autonSpeed, -70 + offset, -10, -40, 2, 2, -40, 0.45, 3300, true, false);
    }
}
