package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SwerveSubsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Retention;

@TeleOp
public class testmode extends OpMode
{

    SparkFunOTOS odometry;
    private double flTgt, frTgt, rlTgt, rrTgt;
//    private SwerveSubsystem swerve;
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private Servo frontLeftServo, frontRightServo, backLeftServo, backRightServo;

    private Servo leftHood, rightHood;

    private AnalogInput frontLeftAnalog, frontRightAnalog, backLeftAnalog, backRightAnalog;

    private DcMotorEx turret;
    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor"); // Ctrl h 3
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor"); // Exp h 1
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor"); // Ctrl h 0
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor"); // Exp h 2

//        swerve = new SwerveSubsystem();
//        swerve.init(hardwareMap);  // Pass OTOS for field-centric

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

//        frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo"); // Ctrl h 1
//        frontRightServo = hardwareMap.get(Servo.class, "frontRightServo"); // Exp h 5
//        backLeftServo = hardwareMap.get(Servo.class, "backLeftServo"); // Ctrl h 2
//        backRightServo = hardwareMap.get(Servo.class, "backRightServo"); // Exp h 4

//        frontLeftServo.setPosition(0);
//        backLeftServo.setPosition(0);
//        frontRightServo.setPosition(0);
//        backRightServo.setPosition(0);

        odometry = hardwareMap.get(SparkFunOTOS.class, "otos");
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.calibrateImu();
        odometry.setAngularScalar(1);
        odometry.setLinearScalar(1);
        odometry.setOffset(new SparkFunOTOS.Pose2D(0, -3.3105, -90));

        odometry.resetTracking();
        odometry.begin();

        odometry.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        turret = hardwareMap.get(DcMotorEx.class, "turretMotor");

        leftHood = hardwareMap.get(Servo.class, "leftHood"); //
        leftHood.setDirection(Servo.Direction.REVERSE);
        rightHood = hardwareMap.get(Servo.class, "rightHood"); //

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

    double xIntegral = 0;
    double yIntegral = 0;

    double lastXPos = 0;
    double lastYPos = 0;

    ElapsedTime dt = new ElapsedTime();

    @Override
    public void loop()
    {
        if (gamepad1.dpad_up)
        {
            flTgt = flOffset;
            rlTgt=  blOffset;
            rrTgt = brOffset;
            frTgt = frOffset;
        }

        leftHood.setPosition(0);
        rightHood.setPosition(0);

        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastB = gamepad1.b;

        telemetry.addData("fl: ", flTgt);
        telemetry.addData("fr: ", frTgt);
        telemetry.addData("bl: ", rlTgt);
        telemetry.addData("br: ", rrTgt);

        SparkFunOTOS.Pose2D position = odometry.getPosition();

        telemetry.addData("x position: ", (position.x));
        telemetry.addData("y position: ", (position.y));

//        telemetry.addData("x position w/ integral: ", (position.x * 1.2) + (0.02022 * xIntegral));
//        telemetry.addData("y position w/ integral: ", (position.y * 1.2) + (0.02022 * yIntegral));

//        xIntegral += Math.abs(position.x - lastXPos);
//        yIntegral += Math.abs(position.y - lastYPos);
//
//        lastXPos = position.x;
//        lastYPos = position.y;
//
//
//        telemetry.addData("x integral", xIntegral);
//        telemetry.addData("y integral", yIntegral);

        telemetry.addData("h position: ", position.h);

        telemetry.update();

        dt.reset();}
}