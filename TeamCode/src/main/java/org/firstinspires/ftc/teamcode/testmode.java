package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SwerveSubsystem;

@TeleOp
public class testmode extends OpMode
{

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

    @Override
    public void loop()
    {
        // 108 forward
        //

        if (gamepad1.a)
        {
            frontLeftServo.setPosition(1);
        }
        else
        {
            frontLeftServo.setPosition(0);
        }

        if (gamepad1.x)
        {
            frontRightServo.setPosition(1);
        }
        else
        {
            frontRightServo.setPosition(0);
        }

        if (gamepad1.y)
        {
            backRightServo.setPosition(1);
        }
        else
        {
            backRightServo.setPosition(0);
        }

        if (gamepad1.b)
        {
            backLeftServo.setPosition(1);
        }
        else
        {
            backLeftServo.setPosition(0);
        }

//        if (gamepad1.a)
//        {
//            frontLeftMotor.setPower(0.6);
//            frontLeftServo.setPower(1);
//        }
//        else
//        {
//            frontLeftMotor.setPower(0);
//            frontLeftServo.setPower(0);
//        }
//        if (gamepad1.b)
//        {
//            frontRightMotor.setPower(0.2);
//            frontRightServo.setPower(0.2);
//        }
//        if (gamepad1.x)
//        {
//            backLeftMotor.setPower(0.6);
//            backLeftServo.setPower(1);
//        }
//        else
//        {
//            backLeftMotor.setPower(0);
//            backLeftServo.setPower(0);
//        }
//        if (gamepad1.y)
//        {
//            backRightMotor.setPower(0.2);
//            backRightServo.setPower(0.2);
//        }
//
//        if (gamepad1.dpad_up) {
//            frontLeftMotor.setPower(0.3);
//        }
//
//        if (gamepad1.dpad_left) {
//            frontRightMotor.setPower(0.3);
//        }
//
//        if (gamepad1.dpad_right) {
//            backLeftMotor.setPower(0.3);
//        }
//
//        if (gamepad1.dpad_down) {
//            backRightMotor.setPower(0.3);
//        }

//        telemetry.addData("Wheels (FL/FR/BL/BR)", "%.0f° %.0f° %.0f° %.0f°",
//                swerve.getFLAngle(), swerve.getFRAngle(),
//                swerve.getBLAngle(), swerve.getBRAngle());
//        telemetry.update();
    }
}