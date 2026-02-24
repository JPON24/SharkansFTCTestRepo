package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.HashMap;

public class MoveCommandVGS {
    WorkingSwerve dt = new WorkingSwerve();
    CommandSystem command = new CommandSystem();
    SharkDrive shark = new SharkDrive();
    ElapsedTime timeout = new ElapsedTime();
    private LinearOpMode opMode;

    VirtualGoalShooter shooter = new VirtualGoalShooter();
    floatingIntake intake = new floatingIntake();


    public void init(HardwareMap hwMap, boolean isAuton, LinearOpMode opMode, boolean blue, boolean far) {
        this.opMode = opMode;
        dt.init(hwMap);
        shark.init(hwMap, isAuton);

        // Init VGS with the OTOS from SharkDrive (single shared instance)
        shooter.init(hwMap, shark.odometry);
        shooter.switchAlliance(blue, far, false);
        shooter.spinUpShooter();

        intake.init(hwMap);
    }

    public void MoveToPosition(double speed, double tgtX, double tgtY, double rot, double d, int axis,
                               boolean intaking, boolean outtaking, boolean shooterOn) {
        command.ResetMap();
        HashMap<Character, Boolean> localCopy;

        command.SetElementFalse('m');
        command.SetElementFalse('s');

        shark.initErrX = tgtX - shark.odometry.getPosition().x;
        shark.initErrY = tgtY - shark.odometry.getPosition().y;

        shark.DihdometryDihtrol2(speed, tgtX, tgtY, rot, d, axis);

        if (shooterOn) {
            shooter.spinUpShooter();
        } else {
            shooter.stopShooter();
        }

        intake.intake(intaking);
        intake.outtake(outtaking);

        localCopy = command.GetMap();
        timeout.reset();

        while (!command.GetBoolsCompleted() && timeout.seconds() < 5 && opMode.opModeIsActive()) {
            shooter.update();

            for (Character key : localCopy.keySet()) {
                switch (key) {
                    case 'm':
                        if (shark.GetStopBoolsCompleted()) {
                            shark.DihdometryDihtrol2(0, tgtX, tgtY, rot, d, axis);
                            command.SetElementTrue('m');
                            break;
                        } else if (shark.GetBoolsCompleted()) {
                            shark.DihdometryDihtrol2(0, tgtX, tgtY, rot, d, axis);
                            command.SetElementTrue('m');
                            break;
                        } else {
                            shark.DihdometryDihtrol2(speed, tgtX, tgtY, rot, d, axis);
                            command.SetElementFalse('m');
                            break;
                        }

                    case 's':
                        if (shooterOn && shooter.isReadyToShoot()) {
                            command.SetElementTrue('s');
                        } else if (!shooterOn) {
                            command.SetElementTrue('s');
                        } else {
                            command.SetElementFalse('s');
                        }
                        break;
                }
            }
        }

        if (intaking) intake.intake(false);
        if (outtaking) intake.outtake(false);
    }

    public boolean GetCommandState() {
        return command.GetBoolsCompleted();
    }
}
