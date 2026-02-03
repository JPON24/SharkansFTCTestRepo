package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class floatingIntake {

    DcMotorSimple intakeMotor; // Exp hub port 2
    DcMotorSimple transferMotor; // Exp port 3

    double transferIntakeSpeed = -0.14;

    double transferSpeed = -1.0;

    double intakeSpeed = 1.0;



    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorSimple.class, "intakeMotor"); // exp hub port 2
        transferMotor = hwMap.get(DcMotorSimple.class, "transferMotor"); // ctrl hub port 2
    }

    public void intake(boolean runIntake) {

        if (runIntake) {
            intakeMotor.setPower(intakeSpeed);
            transferMotor.setPower(transferIntakeSpeed);
        } else {
            intakeMotor.setPower(0);
            transferMotor.setPower(0);
        }
    }

    public void outtake(boolean runOuttake) {

        if (runOuttake) {
            transferMotor.setPower(transferSpeed);
            intakeMotor.setPower(intakeSpeed);
        } else {
            transferMotor.setPower(0);
            intakeMotor.setPower(0);
        }
    }

    public void outFront(boolean runOutFront) {
        if (runOutFront) {
            transferMotor.setPower(-transferSpeed);
            intakeMotor.setPower(-intakeSpeed);
        } else {
            transferMotor.setPower(0);
            intakeMotor.setPower(0);
        }
    }

}

