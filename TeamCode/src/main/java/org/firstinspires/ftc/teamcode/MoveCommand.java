package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveCommand  {
    SwerveSubsystem dt = new SwerveSubsystem();
    CommandSystem command = new CommandSystem();
    SharkDrive shark = new SharkDrive();
    ElapsedTime timeout = new ElapsedTime();

    ShooterSubsystem shooter = new ShooterSubsystem();
    floatingIntake intake = new floatingIntake();

    public void init(HardwareMap hwMap, boolean isAuton)
    {
        dt.init(hwMap);
        shooter.initSystem(hwMap, shark.odometry, 0);
        intake.init(hwMap);
        shark.init(hwMap, isAuton);
    }

    public void MoveToPosition(double speed, double tgtX, double tgtY, double rot, double d, int axis, double turretAngle, double hoodAngle, int RPM, boolean intaking, boolean outtaking)
    {
        // reset for next command
        command.ResetMap();
        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();

        // movement, will be driven by s1.GetBoolsCompleted()
        command.SetElementFalse('m');
        command.SetElementFalse('t'); // turret
        command.SetElementFalse('s'); // shooter

        shark.OdometryControl(speed,tgtX,tgtY,rot,d,axis);
        shooter.setTargetRPM(RPM); // sets bang bang tgt initially
        shooter.turnToAngle(turretAngle, false); // sets tgt angle to ensure that it is not instanlty checked as true

        shooter.setHoodPosition(hoodAngle); // not controlled as cmd because servos return their tgt position not current position with getPosition()

        intake.intake(intaking);
        intake.outtake(outtaking);

        localCopy = command.GetMap();

        timeout.reset();

        while (!command.GetBoolsCompleted()) {
            shooter.BangBang(); // continue to run bang bang updates every iter
            // for every key (m, e, a, c, w)
            for (Character key : localCopy.keySet()) {
                // like an if else but more efficient
                // if a subsystem has reached it's position, set it to complete
                // otherwise, set it to false and stop moving it
                switch (key) {
                    case 'm': // if looking at move
                        if (shark.GetStopBoolsCompleted()) { // for precise movements
                            shark.OdometryControl(0, tgtX, tgtY, rot, d, axis);
                            command.SetElementTrue('m');
                            // dt.FieldOrientedTranslate(0,0,0,0);
                            break;
                        } else if (shark.GetBoolsCompleted()) { // for less precise movements
                            shark.OdometryControl(speed, tgtX, tgtY, rot, d, axis);
                            command.SetElementTrue('m');
                            break;
                        } else {
                            shark.OdometryControl(speed, tgtX, tgtY, rot, d, axis);
                            command.SetElementFalse('m');
                            break;
                        }
                    case 's': // if looking at shooter
                        if (shooter.IsAtTgtRPM()) {
                            command.SetElementTrue('s');
                        }
                    case 't': // if looking at turret
                        if (shooter.IsAtCorrectTurretPos()) {
                            command.SetElementTrue('t');
                            shooter.stopTurret();
                        }
                        else
                        {
                            shooter.turnToAngle(turretAngle, false);
                        }
                }
                if (timeout.seconds() > 5) { // if command somehow broke
                    break;
                }
            }
        }
    }

    public boolean GetCommandState()
    {
        return command.GetBoolsCompleted();
    }
}