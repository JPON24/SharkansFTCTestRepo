package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SwerveSubsystem;

import java.lang.annotation.Retention;

@TeleOp
public class testmode extends OpMode
{


    private double flTgt, frTgt, rlTgt, rrTgt;
//    private SwerveSubsystem swerve;
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private Servo frontLeftServo, frontRightServo, backLeftServo, backRightServo;
    private AnalogInput frontLeftAnalog, frontRightAnalog, backLeftAnalog, backRightAnalog;
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor"); // Ctrl h 3
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor"); // Exp h 1
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor"); // Ctrl h 0
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor"); // Exp h 2

//        swerve = new SwerveSubsystem();
//        swerve.init(hardwareMap);  // Pass OTOS for field-centric

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo"); // Ctrl h 1
        frontRightServo = hardwareMap.get(Servo.class, "frontRightServo"); // Exp h 5
        backLeftServo = hardwareMap.get(Servo.class, "backLeftServo"); // Ctrl h 2
        backRightServo = hardwareMap.get(Servo.class, "backRightServo"); // Exp h 4

        frontLeftServo.setPosition(0);
        backLeftServo.setPosition(0);
        frontRightServo.setPosition(0);
        backRightServo.setPosition(0);
    }

    boolean lastA = false;
    boolean lastX = false;
    boolean lastY = false;
    boolean lastB = false;
    /*
    fl -> 0.22 - 69.3 degrees right - 245.7 degrees right
    fr -> 0.42 - 132.3 degrees right - 182.7 degres left
    bl -> 0.16 - 50.4 degrees right, 264.4 degrees left
    br -> 0.5 - 157.5 deg each way

    max angle = 315 degrees
     */

    double flOffset = 0.22;
    double frOffset = 0.42;
    double blOffset = 0.16;
    double brOffset = 0.5;


    public double GetAngle(double position, double offset)
    {
        return (offset - position) * 315;
    }

    @Override
    public void loop()
    {
        // 315 degree range
        if (gamepad1.a && !lastA) {
            flTgt += 0.02;

            if (flTgt > 1)
            {
                flTgt = 0;
            }
            frontLeftServo.setPosition(flTgt);
        }

        if (gamepad1.x && !lastX)
        {
            frTgt += 0.02;

            if (frTgt > 1)
            {
                frTgt = 0;
            }
            frontRightServo.setPosition(frTgt);
        }

        if (gamepad1.y && !lastY) {
            rrTgt += 0.02;

            if (rrTgt > 1)
            {
                rrTgt = 0;
            }
            backRightServo.setPosition(rrTgt);
        }

        if (gamepad1.b && !lastB) {
            rlTgt += 0.02;
            if (rlTgt > 1)
            {
                rlTgt = 0;
            }
            backLeftServo.setPosition(rlTgt);
        }

        if (gamepad1.dpad_up)
        {
            flTgt = flOffset;
            rlTgt=  blOffset;
            rrTgt = brOffset;
            frTgt = frOffset;

            frontLeftServo.setPosition(flOffset);
            frontRightServo.setPosition(frOffset);
            backLeftServo.setPosition(blOffset);
            backRightServo.setPosition(brOffset);
        }

        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastB = gamepad1.b;

        telemetry.addData("fl: ", flTgt);
        telemetry.addData("fr: ", frTgt);
        telemetry.addData("bl: ", rlTgt);
        telemetry.addData("br: ", rrTgt);

        telemetry.addData("fl deg: ", GetAngle(flTgt, flOffset));
        telemetry.addData("fr deg: ", GetAngle(frTgt, frOffset));
        telemetry.addData("bl deg: ", GetAngle(rlTgt, blOffset));
        telemetry.addData("br deg: ", GetAngle(rrTgt, brOffset));
        telemetry.update();
    }
}