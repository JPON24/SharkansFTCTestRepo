package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@TeleOp
public class AutoShooter extends OpMode
{
    /*
    Shooter tuning

    Long shot


     */


//    floatingIntake intakeObj = new floatingIntake();
    AprilTagLimelight limeLight = new AprilTagLimelight();
    DcMotorEx leftShooter = null;
    DcMotorEx rightShooter = null;
    DcMotorEx turretMotor = null;
    Servo leftHood = null;
    Servo rightHood = null;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime dt = new ElapsedTime();

    public enum ShootState {
        FAR_LOB_SHOT,
        FAR_HARD_SHOT,
        MEDIUM_SHOT,
        CLOSE_SHOT,
        NO_SHOT,
    }
    double integralSum = 0.0;
    double lastError = 0.0;
    double kP = 0.01;
    double kI = 0;
    double kD = 0.00; //

    double hoodPosition = 0.2;

    double filterStrength = 0.8; // we can tune this to make it smooth & slow, or kinda FREAKY & fast.
    double deadband = 3; // This just says if the error is less than this. Just don't bother moving. It's going to save power and still maintain accuracy.
    double maxPower = 0.7;
    double maxDeltaPower = 0.03; // Basically prevents the turret from imploding into a black hole. (See me for more info).

    double turretMinSpeed = 0.1; // 0.3

    double lastFilteredTx = 0;
    double lastOutput = 0;

    // Safety Limits - PLACEHOLDER VALUES, MUST BE TUNED

    final double TICKS_PER_REV = 145.1;
    final double GEAR_RATIO = 5.0;
    final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    final int TURRET_MAX_TICKS = (int)(235 * TICKS_PER_DEGREE);  // ≈ +474 ticks (left limit) // full max is 235
    final int TURRET_MIN_TICKS = (int)(-125 * TICKS_PER_DEGREE); // ≈ -252 ticks (right limit) full max is -125

    // Shooter PLUH
    final double COUNTS_PER_MOTOR_REV = 28.0;
    final double GEAR_REDUCTION = 1;
    final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;

    private AutoShooter.ShootState currentHoodState = AutoShooter.ShootState.NO_SHOT;

    double alternativeAngle = 0;

    double targetRPM = 0;

    enum TurretState {
        TRACKING,
        UNWINDING
    }
    TurretState currentState = TurretState.TRACKING;
    double unwindTargetAngle = 0;
    double unwindStartHeading = 0; // To track robot rotation
    SparkFunOTOS otos;

    @Override
    public void init() {
        limeLight.init(hardwareMap, 0);

//        intakeObj.init(hardwareMap);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor"); // Ctrl Hub 3
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        leftShooter = hwMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter"); // Exp Motor Port 3
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(100, 1, 10, 1));


        leftHood = hardwareMap.get(Servo.class, "leftHood"); // Exp Port 2
        rightHood = hardwareMap.get(Servo.class, "rightHood"); // Exp Port 3

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(new SparkFunOTOS.Pose2D(0,-3.74016,0));
        otos.calibrateImu();

        timer.reset();
    }

    boolean lastRightBumperState = false;
    boolean lastLeftBumperState = false;

    @Override
    public void loop()
    {
        boolean shootButtonPressed = gamepad1.a;
        boolean hardShotPressed = gamepad1.b;

        boolean leftDpad = gamepad1.dpad_left;
        boolean rightDpad = gamepad1.dpad_right;

        boolean upDpad = gamepad1.dpad_up;
        boolean downDpad = gamepad1.dpad_down;

        boolean rightBumperPressed = gamepad1.right_bumper;
        boolean leftBumperPressed = gamepad1.left_bumper;

        if (rightBumperPressed && !lastRightBumperState)
        {
            if (targetRPM + 100 < 6000)
            {
                targetRPM += 100;
            }
        }
        else if (leftBumperPressed && !lastLeftBumperState)
        {
            if (targetRPM - 100 > 0)
            {
                targetRPM -= 100;
            }
        }

        setFlywheelRPM(targetRPM);

        lastRightBumperState = rightBumperPressed;
        lastLeftBumperState = leftBumperPressed;

        telemetry.addData("tgtRPM", targetRPM);
        telemetry.addData("curRPM", getFlywheelRPM());
        telemetry.addData("turret Position", turretMotor.getCurrentPosition());
        telemetry.addData("turretAngle", getTurretAngle());
        telemetry.addData("alt ticks", alternativeAngle);
        telemetry.addData("State", currentState);
        telemetry.update();
//        update(shootButtonPressed, hardShotPressed);

//        if (leftDpad) {
//            intakeObj.intake(true);
//        }
//        else if (rightDpad) {
//            intakeObj.outtake(true);
//        } else {
//            intakeObj.intake(false);
//            intakeObj.outtake(false);
//        }

        if (shootButtonPressed) {
            update(true, false);
        } else {
            update(false, false);
        }

        if (hardShotPressed) {
            update(false, true);
        } else {
            update(false, false);
        }

        turnTurretBlue();
    }

    public void update(boolean isShootButtonPressed, boolean isHardShotPressed) {

        double currentDistance = limeLight.GetDistance();

        final double CLOSE_LIMIT = 40.0;
        final double MEDIUM_LIMIT = 55.0;

        if (isShootButtonPressed) {

            if (currentDistance > 0 && currentDistance < CLOSE_LIMIT) {
                currentHoodState = AutoShooter.ShootState.CLOSE_SHOT;
            } else if (currentDistance >= CLOSE_LIMIT && currentDistance < MEDIUM_LIMIT) {
                currentHoodState = AutoShooter.ShootState.MEDIUM_SHOT;
            } else if (currentDistance >= MEDIUM_LIMIT) {

                if (isHardShotPressed) {
                    currentHoodState = AutoShooter.ShootState.FAR_HARD_SHOT;
                } else {
                    currentHoodState = AutoShooter.ShootState.FAR_LOB_SHOT;
                }
            } else {

                currentHoodState = AutoShooter.ShootState.NO_SHOT;
            }
        } else {

            currentHoodState = AutoShooter.ShootState.NO_SHOT;
        }

        updateShooter();
    }

    public void turnTurretBlue() {
        // UNWINDING takes priority over tracking - check FIRSTWLJORJEWIJF!!
        if (currentState == TurretState.UNWINDING) {
            // Field-Centric Unwinding - compensate for robot rotation
            double currentHeading = Math.toDegrees(otos.getPosition().h);  // Read actual heading
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;

            turnToAngle(dynamicTarget, false);  // false = don't double-negate for Blue

            // Check if we've reached the target
            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;  // Skip all tracking logic while unwinding
        }

        // Normal AprilTag tracking logic
        double tx = limeLight.GetTX();

        // should make it just stop if it sees wrong april tag
        if (limeLight.GetLimelightId() != 21) { // Lowkey not sure what this does for sorting APRILTAGLOL!! Obelisk btw. 20 is the blue
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = 0;
            timer.reset();
            return;
        }

        double dt = timer.seconds();
        if (dt <= 0) dt = 0.001;

        double filteredTx = filterStrength * lastFilteredTx + (1 - filterStrength) * tx;
        lastFilteredTx = filteredTx; // lowkey, all the stupid spinners I see are super smooth. I want ours to be too!!!! :)
        // TS filtered TX basically just micro correct how the PID is going to oscilate between the april tag since we MOVE!!


        if (Math.abs(filteredTx) < deadband) {
            turretMotor.setPower(0);
            lastOutput = 0;
            integralSum = 0;
            return;
        }

        double error = filteredTx;

        double derivative = (error - lastError) / dt;
        lastError = error;

        integralSum += error * dt;

        double output = kP * error + kI * integralSum + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;
        timer.reset();

        output = Math.max(-maxPower, Math.min(maxPower, output));

        int currentPos = turretMotor.getCurrentPosition();
        // At METHX: block if trying to go right (output < 0 → -output > 0 → going positive)
        // At METHN: block if trying to go left (output > 0 → -output < 0 → going negative)
        boolean hitMax = (currentPos > TURRET_MAX_TICKS && -output > 0);
        boolean hitMin = (currentPos < TURRET_MIN_TICKS && -output > 0);  // SWAPPED THIS

        if (hitMax || hitMin) {
            turretMotor.setPower(0);

            // Calculate alternative angle by going 360° the other way
            double currentAngle = getTurretAngle();
            alternativeAngle = hitMax ? (currentAngle - 360) : (currentAngle + 360);


            // With proper ±180° limits, the alternative should always be in range
            double altTicks = alternativeAngle * TICKS_PER_DEGREE;
            if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                currentState = TurretState.UNWINDING;
                unwindTargetAngle = alternativeAngle;
                unwindStartHeading = Math.toDegrees(otos.getPosition().h);  // Capture current heading
            }
            return;
        }

        // Add minimum speed to overcome friction
        if (output > 0) {
            output += turretMinSpeed;
        }

        if (output < 0) {
            output -= turretMinSpeed;
        }

        // lowkey, if it's backwards, just reverse the output!!
        turretMotor.setPower(-output);
    }
    public void turnTurretRed() {
        // UNWINDING takes priority over tracking - check first! TURSTWIJFOIWJE
        if (currentState == TurretState.UNWINDING) {
            // Field-Centric Unwinding - compensate for robot rotation
            double currentHeading = Math.toDegrees(otos.getPosition().h);  // Reeeeeeed actual heading
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;

            turnToAngle(dynamicTarget, true);  // true = negate for Red

            // Check if we've reached the target
            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;  // Skip all tracking logic while unwinding
        }

        // Normal AprilTag tracking logic
        double tx = limeLight.GetTX();

        // should make it just stop if it sees wrong april tag
        if (limeLight.GetLimelightId() != 24) { // Lowkey not sure what this does for sorting APRILTAGLOL!!
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = 0;
            timer.reset();
            return;
        }

        double dt = timer.seconds();
        if (dt <= 0) dt = 0.001;

        double filteredTx = filterStrength * lastFilteredTx + (1 - filterStrength) * tx;
        lastFilteredTx = filteredTx; // lowkey, all the stupid spinners I see are super smooth. I want ours to be too!!!! :)
        // TS filtered TX basically just micro correct how the PID is going to oscilate between the april tag since we MOVE!!


        if (Math.abs(filteredTx) < deadband) {
            turretMotor.setPower(0);
            lastOutput = 0;
            integralSum = 0;
            return;
        }

        double error = filteredTx;

        double derivative = (error - lastError) / dt;
        lastError = error;

        integralSum += error * dt;

        double output = kP * error + kI * integralSum + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;
        timer.reset();

        // Normul jrrrking logic
        output = Math.max(-maxPower, Math.min(maxPower, output));

        // Check if we hit limits - no negation here, output is applied directly
        int currentPos = turretMotor.getCurrentPosition();
        boolean hitMax = (currentPos > TURRET_MAX_TICKS && output > 0);  // Block positive output (going right) at max
        boolean hitMin = (currentPos < TURRET_MIN_TICKS && output < 0);  // Block negative output (going left) at min

        if (hitMax || hitMin) {
            turretMotor.setPower(0);

            // Calculate 360° alternative - with proper asymmetric 360° range, always in bounds
            double currentAngle = getTurretAngle();
            alternativeAngle = hitMax ? (currentAngle - 360) : (currentAngle + 360);

            double altTicks = alternativeAngle * TICKS_PER_DEGREE;
            if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                currentState = TurretState.UNWINDING;
                unwindTargetAngle = alternativeAngle;
                unwindStartHeading = Math.toDegrees(otos.getPosition().h);
            }
            return;  // Don't apply any power this loop
        }

        // Add minimum speed to overcome friction
        if (output > 0) {
            output += turretMinSpeed;
        }

        if (output < 0) {
            output -= turretMinSpeed;
        }

        // lowkey, if it's backwards, just reverse the output!!
        turretMotor.setPower(output);
    }

    public void setFlywheelRPM(double targetRPM) {
        double ticksPerSecond = (targetRPM / 60.0) * COUNTS_PER_WHEEL_REV;
        rightShooter.setVelocity(ticksPerSecond);
    }

    public double getFlywheelRPM() {
        return (rightShooter.getVelocity() / COUNTS_PER_WHEEL_REV) * 60.0;
    }

    public boolean isFlywheelAtSpeed(double targetRPM, double tolerance) {
        return Math.abs(getFlywheelRPM() - targetRPM) < tolerance;
    }

    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void turnToAngle(double targetAngle, boolean negateOutput) {

        double currentAngle = getTurretAngle();

        double error = targetAngle - currentAngle;

        double dtSeconds = timer.seconds();
        timer.reset();
        if (dtSeconds <= 0) dtSeconds = 0.001;

        integralSum += error * dtSeconds;
        double maxIntegral = 1.0; // tune as needed yuh?
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));

        double derivative = (error - lastError) / dtSeconds;
        lastError = error;

        double output = kP * error + kI * integralSum + kD * derivative;

        double delta = output - lastOutput;
        if (Math.abs(delta) > maxDeltaPower) {
            output = lastOutput + Math.signum(delta) * maxDeltaPower;
        }
        lastOutput = output;

        output = Math.max(-maxPower, Math.min(maxPower, output));

        // Apply negation for blue alliance if needed
        if (negateOutput) {
            output = -output;
        }

        turretMotor.setPower(output);
    }


    public void updateShooter() {
        switch(currentHoodState) {
            case FAR_LOB_SHOT:
                SetHoodPosition(0.40);
                // setFlywheelRPM(3150);
                break;

            case FAR_HARD_SHOT:
                SetHoodPosition(0.2);
                // setFlywheelRPM(3100);
                break;

            case MEDIUM_SHOT:
                SetHoodPosition(0.30);
                // setFlywheelRPM(3000);
                break;

            case CLOSE_SHOT:
                SetHoodPosition(0.40);
                // setFlywheelRPM(2700);
                break;

            case NO_SHOT:
                SetHoodPosition(0.30);
                // setFlywheelRPM(0);
                break;
        }
    }

    public void SetHoodPosition(double value)
    {
        double position = value;
        leftHood.setPosition(position);
        rightHood.setPosition(position);
    }
}