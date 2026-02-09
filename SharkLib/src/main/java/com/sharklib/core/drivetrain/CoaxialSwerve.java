package com.sharklib.core.drivetrain;

import static android.icu.lang.UCharacter.GraphemeClusterBreak.L;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.sharklib.core.hardware.Motor;
import com.sharklib.core.hardware.SmartCRPServo;
import com.sharklib.core.control.PIDController;
import com.sharklib.core.util.math.geometry.Vector2D;
import com.sharklib.core.util.math.geometry.RobotGeometry;

public class CoaxialSwerve {

    RobotGeometry dimensions;

    Vector2D frontLeft;
    Vector2D frontRight;
    Vector2D backLeft;
    Vector2D backRight;

    PIDController frontLeftPID;
    PIDController frontRightPID;
    PIDController backLeftPID;
    PIDController backRightPID;

    SmartCRPServo frontLeftServo;
    SmartCRPServo frontRightServo;
    SmartCRPServo backLeftServo;
    SmartCRPServo backRightServo;

    Motor frontLeftMotor;
    Motor frontRightMotor;
    Motor backLeftMotor;
    Motor backRightMotor;


    public CoaxialSwerve(HardwareMap hwMap,
                         String flS_name, String frS_name, String blS_name, String brS_name,
                         String flM_name, String frM_name, String blM_name, String brM_name)
    {
        this.frontLeftServo = hwMap.get(SmartCRPServo.class, flS_name);
        this.frontRightServo = hwMap.get(SmartCRPServo.class, frS_name);
        this.backLeftServo = hwMap.get(SmartCRPServo.class, blS_name);
        this.backRightServo = hwMap.get(SmartCRPServo.class, brS_name);

        this.frontLeftMotor = hwMap.get(Motor.class, flM_name);
        this.frontRightMotor = hwMap.get(Motor.class, frM_name);
        this.backLeftMotor = hwMap.get(Motor.class, blM_name);
        this.backRightMotor = hwMap.get(Motor.class, brM_name);

    }

    public void CoaxialSwerveConstants(double flP, double flI, double flD, double flF,
                                  double frP, double frI, double frD, double frF,
                                  double blP, double blI, double blD, double blF,
                                  double brP, double brI, double brD, double brF,
                                       double width, double length, double height)
    {
        this.frontLeftPID = new PIDController(flP, flI, flD, flF);
        this.frontRightPID = new PIDController(frP, frI, frD, frF);
        this.backLeftPID = new PIDController(blP, blI, blD, blF);
        this.backRightPID = new PIDController(brP, brI, brD, brF);

        this.dimensions = new RobotGeometry(width, length, height);
    }



    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void drive(double x_cmd, double y_cmd, double turn_cmd, double heading) {

        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stop();
            return;
        }

        double rotX = -x_cmd * Math.cos(heading) + y_cmd * Math.sin(heading);
        double rotY = -x_cmd * Math.sin(heading) - y_cmd * Math.cos(heading);

        x_cmd = rotX;
        y_cmd = rotY;

        double y_fr = y_cmd + turn_cmd * dimensions.length;
        double x_fr = x_cmd + turn_cmd * dimensions.width;

        double y_fl = y_cmd - turn_cmd * dimensions.length;
        double x_fl = x_cmd + turn_cmd * dimensions.width;

        double y_rl = y_cmd - turn_cmd * dimensions.length;;
        double x_rl = x_cmd - turn_cmd * dimensions.width;

        double y_rr = y_cmd + turn_cmd * dimensions.length;;
        double x_rr = x_cmd - turn_cmd * dimensions.width;

        Vector2D fl = new Vector2D(x_fl, y_fl);
        Vector2D fr = new Vector2D(x_fr, y_fr);
        Vector2D rl = new Vector2D(x_rl, y_rl);
        Vector2D rr = new Vector2D(x_rr, y_rr);
    }
}