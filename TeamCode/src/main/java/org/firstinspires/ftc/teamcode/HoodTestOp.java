package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.util.gamepad.EnhancedGamepad;


@TeleOp
public class HoodTestOp extends OpMode{

    SparkFunOTOS otos = null;

    VirtualGoalShooter shooter = new VirtualGoalShooter();

    Servo leftHood = null;
    Servo rightHood = null;

    EnhancedGamepad g1 = null;

    double hoodPosition = 0;


    @Override
    public void init() {

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(new SparkFunOTOS.Pose2D(0, -3.74016, 0));
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        otos.calibrateImu();

        shooter = new VirtualGoalShooter();
        shooter.init(hardwareMap, otos);

        leftHood = hardwareMap.get(Servo.class, "leftHood"); //
        leftHood.setDirection(Servo.Direction.REVERSE);
        rightHood = hardwareMap.get(Servo.class, "rightHood"); //

        shooter.switchAlliance(true, false, true);

        g1 = new EnhancedGamepad(gamepad1);

    }


    @Override
    public void loop() {

        g1.update();

        shooter.update();

        if (g1.dpad_up.wasJustPressed()) {
            hoodPosition += 0.05;
        } else if (g1. dpad_down.wasJustPressed()) {
            hoodPosition -= 0.05;
        }

        leftHood.setPosition(hoodPosition);
        rightHood.setPosition(hoodPosition);


        telemetry.addData("Turret Position", shooter.getTurretDegrees());
        telemetry.addData("RPM (current/target)", "%.0f / %.0f", shooter.getCurrentRPM(), shooter.getTargetRPM());
        telemetry.addData("Hood", "%.2f", hoodPosition);
        telemetry.update();


    }
}
