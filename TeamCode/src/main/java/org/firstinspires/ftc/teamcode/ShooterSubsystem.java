package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterSubsystem {
    
    public enum ShootState {
        FAR_LOB_SHOT,
        FAR_HARD_SHOT,
        MEDIUM_SHOT,
        CLOSE_SHOT,
        NO_SHOT,
    }
    
    public enum TurretState {
        TRACKING,
        UNWINDING
    }
    
    private AprilTagLimelight limeLight = new AprilTagLimelight();
    private DcMotorEx turretMotor = null;
    private DcMotorEx rightShooter = null;
    private Servo leftHood = null;
    private Servo rightHood = null;
    private SparkFunOTOS otos;
    
    private ElapsedTime timer = new ElapsedTime();
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double kP = 0.002;
    private double kI = 0.0;
    private double kD = 0.0;
    
    private double hoodPosition = 0.2;
    private double filterStrength = 0.8;
    private double deadband = 3;
    private double maxPower = 0.7;
    private double maxDeltaPower = 0.03;
    private double turretMinSpeed = 0.1;
    private double lastFilteredTx = 0;
    private double lastOutput = 0;
    
    private final double TICKS_PER_REV = 145.1;
    private final double GEAR_RATIO = 5.0;
    private final double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
    private final int TURRET_MAX_TICKS = (int)(235 * TICKS_PER_DEGREE);
    private final int TURRET_MIN_TICKS = (int)(-125 * TICKS_PER_DEGREE);
    
    private final double COUNTS_PER_MOTOR_REV = 28.0;
    private final double GEAR_REDUCTION = 1;
    private final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION;
    
    private ShootState currentHoodState = ShootState.NO_SHOT;
    private double targetRPM = 0;
    private double alternativeAngle = 0;
    
    private TurretState currentState = TurretState.TRACKING;
    private double unwindTargetAngle = 0;
    private double unwindStartHeading = 0;
    
    public void init(HardwareMap hardwareMap, SparkFunOTOS otosRef) {
        limeLight.init(hardwareMap);
        this.otos = otosRef;
        
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, 
                                         new PIDFCoefficients(100, 1, 10, 1));
        
        leftHood = hardwareMap.get(Servo.class, "leftHood");
        rightHood = hardwareMap.get(Servo.class, "rightHood");
        
        timer.reset();
    }
    
    public void update(boolean isShootButtonPressed, boolean isHardShotPressed) {
        double currentDistance = limeLight.GetDistance();
        final double CLOSE_LIMIT = 40.0;
        final double MEDIUM_LIMIT = 55.0;

        if (isShootButtonPressed) {
            if (currentDistance > 0 && currentDistance < CLOSE_LIMIT) {
                currentHoodState = ShootState.CLOSE_SHOT;
            } else if (currentDistance >= CLOSE_LIMIT && currentDistance < MEDIUM_LIMIT) {
                currentHoodState = ShootState.MEDIUM_SHOT;
            } else if (currentDistance >= MEDIUM_LIMIT) {
                if (isHardShotPressed) {
                    currentHoodState = ShootState.FAR_HARD_SHOT;
                } else {
                    currentHoodState = ShootState.FAR_LOB_SHOT;
                }
            } else {
                currentHoodState = ShootState.NO_SHOT;
            }
        } else {
            currentHoodState = ShootState.NO_SHOT;
        }

        updateHood();
    }
    
    public void turnTurretBLUE() {
        if (currentState == TurretState.UNWINDING) {
            double currentHeading = Math.toDegrees(otos.getPosition().h);
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;

            turnToAngle(dynamicTarget, false);

            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;
        }

        double tx = limeLight.GetTX();

        if (limeLight.GetLimelightId() != 20) {
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = 0;
            timer.reset();
            return;
        }

        double dt = timer.seconds();
        if (dt <= 0) dt = 0.001;

        double filteredTx = filterStrength * lastFilteredTx + (1 - filterStrength) * tx;
        lastFilteredTx = filteredTx;

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
        boolean hitMax = (currentPos > TURRET_MAX_TICKS && -output > 0);
        boolean hitMin = (currentPos < TURRET_MIN_TICKS && -output > 0);

        if (hitMax || hitMin) {
            turretMotor.setPower(0);

            double currentAngle = getTurretAngle();
            alternativeAngle = hitMax ? (currentAngle - 360) : (currentAngle + 360);

            double altTicks = alternativeAngle * TICKS_PER_DEGREE;
            if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                currentState = TurretState.UNWINDING;
                unwindTargetAngle = alternativeAngle;
                unwindStartHeading = Math.toDegrees(otos.getPosition().h);
            }
            return;
        }

        if (output > 0) output += turretMinSpeed;
        if (output < 0) output -= turretMinSpeed;

        turretMotor.setPower(-output);
    }

    public void turnTurretRED() {
        if (currentState == TurretState.UNWINDING) {
            double currentHeading = Math.toDegrees(otos.getPosition().h);
            double headingDelta = currentHeading - unwindStartHeading;
            double dynamicTarget = unwindTargetAngle - headingDelta;

            turnToAngle(dynamicTarget, false);

            if (Math.abs(getTurretAngle() - dynamicTarget) < 5.0) {
                currentState = TurretState.TRACKING;
                timer.reset();
            }
            return;
        }

        double tx = limeLight.GetTX();

        if (limeLight.GetLimelightId() != 24) {
            turretMotor.setPower(0);
            integralSum = 0;
            lastError = 0;
            timer.reset();
            return;
        }

        double dt = timer.seconds();
        if (dt <= 0) dt = 0.001;

        double filteredTx = filterStrength * lastFilteredTx + (1 - filterStrength) * tx;
        lastFilteredTx = filteredTx;

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
        boolean hitMax = (currentPos > TURRET_MAX_TICKS && -output > 0);
        boolean hitMin = (currentPos < TURRET_MIN_TICKS && -output > 0);

        if (hitMax || hitMin) {
            turretMotor.setPower(0);

            double currentAngle = getTurretAngle();
            alternativeAngle = hitMax ? (currentAngle - 360) : (currentAngle + 360);

            double altTicks = alternativeAngle * TICKS_PER_DEGREE;
            if (altTicks >= TURRET_MIN_TICKS && altTicks <= TURRET_MAX_TICKS) {
                currentState = TurretState.UNWINDING;
                unwindTargetAngle = alternativeAngle;
                unwindStartHeading = Math.toDegrees(otos.getPosition().h);
            }
            return;
        }

        if (output > 0) output += turretMinSpeed;
        if (output < 0) output -= turretMinSpeed;

        turretMotor.setPower(-output);
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
        double ticksPerSecond = (targetRPM / 60.0) * COUNTS_PER_WHEEL_REV;
        rightShooter.setVelocity(ticksPerSecond);
    }
    
    private void turnToAngle(double targetAngle, boolean negateOutput) {
        double currentAngle = getTurretAngle();
        double error = targetAngle - currentAngle;

        double dtSeconds = timer.seconds();
        timer.reset();
        if (dtSeconds <= 0) dtSeconds = 0.001;

        integralSum += error * dtSeconds;
        double maxIntegral = 1.0;
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

        if (negateOutput) {
            output = -output;
        }

        turretMotor.setPower(output);
    }

    private void updateHood() {
        switch(currentHoodState) {
            case FAR_LOB_SHOT:
                setHoodPosition(0.40);
                break;
            case FAR_HARD_SHOT:
                setHoodPosition(0.2);
                break;
            case MEDIUM_SHOT:
                setHoodPosition(0.30);
                break;
            case CLOSE_SHOT:
                setHoodPosition(0.40);
                break;
            case NO_SHOT:
                setHoodPosition(0.30);
                break;
        }
    }

    private void setHoodPosition(double value) {
        leftHood.setPosition(value);
        rightHood.setPosition(value);
    }

    // Public getters for telemetry
    public double getTargetRPM() { return targetRPM; }
    public double getCurrentRPM() { 
        return (rightShooter.getVelocity() / COUNTS_PER_WHEEL_REV) * 60.0;
    }
    public double getTurretAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }
    public int getTurretTicks() { return turretMotor.getCurrentPosition(); }
    public TurretState getTurretState() { return currentState; }
}
