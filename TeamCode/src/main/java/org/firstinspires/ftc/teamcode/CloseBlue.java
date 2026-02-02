package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class CloseBlue extends LinearOpMode
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
            Shoot(0);
            SpikeMarkTwo(0);
            Shoot(0);

            GateIntake(0);
            Shoot(0);
            GateIntake(0);
            Shoot(0);

            SpikeMarkOne(0);
            ShootLeave(0);

            break;
        }
    }

    // offset if otos drifts heavily
    private void Shoot(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -2, -36, 30, 1, 3, 60, 0.45, 3200, false, false);
        moveCmd.MoveToPosition(autonSpeed, -2, -36, 30, 1, 4, 60, 0.45, 3200, false, true);
        sleep(500);
    }

    private void ShootLeave(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -2, -36, 30, 1, 2, 60, 0.45, 3200, false, false);
        moveCmd.MoveToPosition(autonSpeed, -2, -36, 30, 1, 4, 60, 0.45, 3200, false, true);
        sleep(500);
        moveCmd.MoveToPosition(autonSpeed, 6, -44, 30, 1, 2, 60, 0.45, 3200, false, false);
    }

    private void SpikeMarkOne(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -30, -37, 30, 1, 2, 60, 0.45, 3200, true, false);
        moveCmd.MoveToPosition(autonSpeed, -38.3, -17.7, 30, 1, 2, 60, 0.45, 3200, true, false);
    }

    private void SpikeMarkTwo(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -54, -46, 30, 1, 2, 60, 0.45, 3200, true, false);
        moveCmd.MoveToPosition(autonSpeed, -60.53, -26.15, 30, 1, 2, 60, 0.45, 3200, true, false);
    }

    private final int gateIntakeTimingMs = 2000;
    private void GateIntake(double offset)
    {
        moveCmd.MoveToPosition(autonSpeed, -56.25, -20, 5, 1, 3, 60, 0.45, 3200, true, true);
        sleep(gateIntakeTimingMs); // tune this to figure out gate intake timing
    }
}
