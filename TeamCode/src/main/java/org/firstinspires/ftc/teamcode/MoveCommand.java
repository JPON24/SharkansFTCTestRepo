//package org.firstinspires.ftc.teamcode;
//
//import java.util.HashMap;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//public class MoveCommand  {
//    SwerveSubsystem dt = new SwerveSubsystem();
//    CommandSystem command = new CommandSystem();
//    SharkDrive shark = new SharkDrive();
//    ElapsedTime timeout = new ElapsedTime();
//
//    int lastA = 0;
//    int lastE = 0;
//
//    boolean usingWrist = false;
//
//    public void init(HardwareMap hwMap, boolean isAuton)
//    {
//        dt.init(hwMap);
//    }
//
//    public void MoveToPosition(double speed, double tgtX, double tgtY, double rot, double d, int axis, double speedA, int tgtA, int tgtE, double roll, double pitch, boolean tgtClaw, char tgtWrist)
//    {
//        // reset for next command
//        command.ResetMap();
//        HashMap<Character, Boolean> localCopy = new HashMap<Character,Boolean>();
//
//        // movement, will be driven by s1.GetBoolsCompleted()
//        command.SetElementFalse('m');
//
//        shark.OdometryControl(speed,tgtX,tgtY,rot,d,axis);
//
//        localCopy = command.GetMap();
//
//        timeout.reset();
//
//        while (!command.GetBoolsCompleted()) {
//            // for every key (m, e, a, c, w)
//            for (Character key : localCopy.keySet()) {
//                // like an if else but more efficient
//                // if a subsystem has reached it's position, set it to complete
//                // otherwise, set it to false and stop moving it
//                switch (key) {
//                    case 'm':
//                        if (shark.GetStopBoolsCompleted()) {
//                            shark.OdometryControl(0, tgtX, tgtY, rot, d, axis);
//                            command.SetElementTrue('m');
//                            // dt.FieldOrientedTranslate(0,0,0,0);
//                            break;
//                        }
//                        else if (shark.GetBoolsCompleted())
//                        {
//                            shark.OdometryControl(speed, tgtX, tgtY, rot, d, axis);
//                            command.SetElementTrue('m');
//                            break;
//                        }
//                        else
//                        {
//                            shark.OdometryControl(speed, tgtX, tgtY, rot, d, axis);
//                            command.SetElementFalse('m');
//                            break;
//                        }
//                }
//            }
//            if (timeout.seconds() > 3.5)
//            {
//                break;
//            }
//        }
//    }
//
//
//
//    public boolean GetCommandState()
//    {
//        return command.GetBoolsCompleted();
//    }
//
//    public boolean UsingWrist()
//    {
//        return usingWrist;
//    }
//}