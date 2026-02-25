package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.SwerveSubsystem;
import org.firstinspires.ftc.teamcode.floatingIntake;

import java.util.HashMap;

/**
 * What I changed for you my glorious king #3
 * Only 3 critical changes from original:
 * 1. Added break statements in switch cases
 * 2. Fixed timeout to break while loop
 * 3. Added else clause for shooter state
 */
public class MoveCommand {
    WorkingSwerve dt = new WorkingSwerve();
    CommandSystem command = new CommandSystem();
    SharkDrive shark = new SharkDrive();
    ElapsedTime timeout = new ElapsedTime();
    private LinearOpMode opMode;

    ShooterSubsystem shooter = new ShooterSubsystem();
    floatingIntake intake = new floatingIntake();

    // Backward-compatible init (no opMode reference)
    public void init(HardwareMap hwMap, boolean isAuton) {
        init(hwMap, isAuton, null);
    }

    public void init(HardwareMap hwMap, boolean isAuton, LinearOpMode opMode) {
        this.opMode = opMode;
        dt.init(hwMap);
        shooter.initSystem(hwMap, shark.odometry, 0);
        intake.init(hwMap);
        shark.init(hwMap, isAuton);
    }

    public void MoveToPosition(double speed, double tgtX, double tgtY, double rot, double d, int axis, double turretAngle, double hoodAngle, int RPM, boolean intaking, boolean outtaking) {
        // reset for next command
        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();

        // movement, will be driven by s1.GetBoolsCompleted()
        command.SetElementFalse('m');
        command.SetElementFalse('t'); // turret
        command.SetElementFalse('s'); // shooter

        shark.initErrX = tgtX - shark.odometry.getPosition().x;
        shark.initErrY = tgtY - shark.odometry.getPosition().y;

        shark.DihdometryDihtrol2(speed,tgtX,tgtY,rot,d,axis);
        shooter.setTargetRPM(RPM); // sets bang bang tgt initially
        shooter.turnToAngle(turretAngle, false);

        shooter.setHoodPosition(hoodAngle);

        intake.intake(intaking);
        intake.outtake(outtaking);

        localCopy = command.GetMap();

        timeout.reset();

        while (!command.GetBoolsCompleted() && timeout.seconds() < 5 && (opMode == null || opMode.opModeIsActive())) {
            shooter.BangBang(); // continue to run bang bang updates every iter
            // for every key (m, e, a, c, w)
            for (Character key : localCopy.keySet()) {
                switch (key) {
                    case 'm': // if looking at move
                        if (shark.GetStopBoolsCompleted()) { // for precise movements
                            shark.DihdometryDihtrol2(0, tgtX, tgtY, rot, d, axis);
                            command.SetElementTrue('m');
                            // dt.FieldOrientedTranslate(0,0,0,0);
                            break;
                        } else if (shark.GetBoolsCompleted()) { // for less precise movements
                            shark.DihdometryDihtrol2(0, tgtX, tgtY, rot, d, axis);
                            command.SetElementTrue('m');
                            break;
                        } else {
                            shark.DihdometryDihtrol2(speed, tgtX, tgtY, rot, d, axis);
                            command.SetElementFalse('m');
                            break;
                        }

                    case 's':
                        if (shooter.IsAtTgtRPM()) {
                            command.SetElementTrue('s');
                        } else {
                            command.SetElementFalse('s');
                        }
                        break;

                    case 't':
                        if (shooter.IsAtCorrectTurretPos()) {
                            command.SetElementTrue('t');
                            shooter.stopTurret();
                        } else {
                            shooter.turnToAngle(turretAngle, false);
                            command.SetElementFalse('t');
                        }
                        break;
                }
            }
        }
    }

    public boolean GetCommandState() {
        return command.GetBoolsCompleted();
    }
}