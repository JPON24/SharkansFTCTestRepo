package org.firstinspires.ftc.teamcode.global.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double speedScalar = 1.0;

    /**
     * Initializes the 4 motors by name.
     * Place this in initialization.
     */
    public void mecanumConstants(HardwareMap hw, String flName, String frName, String blName, String brName) {
        frontLeft = hw.get(DcMotor.class, flName);
        frontRight = hw.get(DcMotor.class, frName);
        backLeft = hw.get(DcMotor.class, blName);
        backRight = hw.get(DcMotor.class, brName);

        // Reverse right side so positive power moves forward
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake mode for precision
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setScalar(double scalar) {
        this.speedScalar = scalar;
    }

    /**
     * Mecanum drive
     * @param x  The strafe input (left_stick_x)
     * @param y  The forward input (-left_stick_y)
     * @param rx The rotation input (right_stick_x)
     */
    public void drive(double x, double y, double rx, double heading) {

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double theta = Math.atan2(rotY, rotX);
        double power = Math.hypot(rotX, rotY);

        // Mecanum Math: offset by 45 degrees (PI/4) to align with wheel rollers
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);

        // Normalize based on the largest trig value to keep speed consistent
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // Use a reverse rotation constant as per your original code
        double rotation = rx * -1;

        // Apply vector math to individual wheels
        double flP = power * cos / max + rotation;
        double frP = power * sin / max - rotation;
        double blP = power * sin / max + rotation;
        double brP = power * cos / max - rotation;

        // Safety Normalization: Prevent powers exceeding 1.0 if we are turning + moving
        if ((power + Math.abs(rotation)) > 1) {
            flP /= power + Math.abs(rotation);
            frP /= power + Math.abs(rotation);
            blP /= power + Math.abs(rotation);
            brP /= power + Math.abs(rotation);
        }

        // Send power * speedScalar
        frontLeft.setPower(flP * speedScalar);
        frontRight.setPower(frP * speedScalar);
        backLeft.setPower(blP * speedScalar);
        backRight.setPower(brP * speedScalar);
    }
}