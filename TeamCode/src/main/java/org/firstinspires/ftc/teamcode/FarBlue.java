package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class FarBlue extends LinearOpMode
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
            Shoot(0);
            SpikeMarkOne(0);

            Shoot(0);
            HumanPlayerZone(0);

            Shoot(0);
            HumanPlayerZone(0);

            ShootLeave();
            break;
        }
    }

    // offset if otos drifts heavily
    private void Shoot(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, 0, 5, 0, 1, 4, -15, 0, 4300, false, false);
        moveCmd.MoveToPosition(autonSpeed, 0, 5, 0, 1, 4, -15, 0, 4300, false, true);
        sleep(500);
    }

    private void ShootLeave()
    {
        Shoot(0);
        moveCmd.MoveToPosition(autonSpeed, -18, 10, 0, 1, 2, 0, 0, 0, false, false);
    }

    private void SpikeMarkOne(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -18, 24, 0, 1, 2, -15, 0, 4300, true, false);
        moveCmd.MoveToPosition(autonSpeed, -48, 24, 0, 1, 2, -15, 0, 4300, true, false);
    }

    private void HumanPlayerZone(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -50, 5, -90, 1, 2, -15, 0, 4300, true, false);
    }
}