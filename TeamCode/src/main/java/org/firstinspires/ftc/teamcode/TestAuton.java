package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class TestAuton extends LinearOpMode
{
    SharkDrive shark = new SharkDrive();
    MoveCommand moveCmd = new MoveCommand();

    private final double autonSpeed = 0.7;

    @Override
    public void runOpMode()
    {
        shark.init(hardwareMap, true);
        moveCmd.init(hardwareMap, true);

        waitForStart();
        while (opModeIsActive())
        {
            // move backward from goal (oriented off of OTOS)
//            if (gamepad1.a)
//            {
//                Shoot(0);
//            }
//            if (gamepad1.b)
//            {
//                Outtake(0);
//            }
//            if (gamepad1.x)
//            {
//                Intake(0);
//            }
//            if (gamepad1.y)
//            {
//                Turret(0);
//            }
//            if (gamepad1.dpad_up)
//            {
//                Hood(0);
//            }
//            if (gamepad1.dpad_up)
//            {
            BasicMove(0);
//            }
        }
    }

    private void BasicMove(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 12, 0, 0, 0, 0, 0, 0, 0, false, false);
        moveCmd.MoveToPosition(autonSpeed, 12, 12, 0, 0, 1, 0, 0, 0, false, false);
        moveCmd.MoveToPosition(autonSpeed, 0, 0, 0, 0, 2, 0, 0, 0, false, false);
        moveCmd.MoveToPosition(autonSpeed, 0, 0, 90, 0, 3, 0, 0, 0, false, false);
    }

    // offset if otos drifts heavily
    private void Shoot(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 0, 0, 0, 0, 4, 0, 0, 3200, false, false);
    }

    private void Outtake(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 0, 0, 0, 0, 4, 0, 0, 0, false, true);
    }

    private void Intake(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 0, 0, 0, 0, 4, 0, 0, 0, true, false);
    }

    private void Turret(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 0, 0, 0, 0, 4, 30, 0, 0, false, false);
    }

    private void Hood(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 0, 0, 0, 0, 4, 0, 0.45, 0, false, false);
    }
}