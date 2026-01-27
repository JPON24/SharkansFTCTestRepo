package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TEST OPMODE for MPC Controller
 * Use this to tune and debug MPC without affecting the main shooter code
 * 
 * B button toggles between Simple MPC (grid search) and Real MPC (QP solver)
 */
@TeleOp(name = "MPC Turret Test", group = "Experimental")
public class MPCTurretTest extends OpMode {

    private DcMotorEx turretMotor;
    private SparkFunOTOS otos;
    private SimpleMPCController simpleMpc;
    private RealMPCController realMpc;
    private ElapsedTime loopTimer = new ElapsedTime();
    
    // Constants (copy from ShooterSubsystem)
    private static final double TICKS_PER_DEGREE = 22.76;
    private static final int TURRET_MAX_TICKS = 12288;
    private static final int TURRET_MIN_TICKS = -12288;
    
    private double targetAngle = 0;
    private boolean lastBumperState = false;
    private boolean lastBState = false;
    private boolean useRealMpc = true;  // Start with Real MPC
    
    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(new SparkFunOTOS.Pose2D(0, -3.74016, 0));
        otos.calibrateImu();
        
        simpleMpc = new SimpleMPCController();
        realMpc = new RealMPCController();
        
        telemetry.addData("Status", "MPC Turret Test Ready");
        telemetry.addData("Mode", "Real MPC (QP Solver)");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        double loopTime = loopTimer.seconds();
        loopTimer.reset();
        
        // Controls
        double stickInput = -gamepad1.left_stick_y;
        if (Math.abs(stickInput) > 0.1) {
            targetAngle += stickInput * 2.0;
            targetAngle = Math.max(-180, Math.min(180, targetAngle));
        }
        
        // Right bumper resets
        if (gamepad1.right_bumper && !lastBumperState) {
            targetAngle = 0;
            simpleMpc.reset();
            realMpc.reset();
        }
        lastBumperState = gamepad1.right_bumper;
        
        // B button toggles MPC type
        if (gamepad1.b && !lastBState) {
            useRealMpc = !useRealMpc;
            simpleMpc.reset();
            realMpc.reset();
        }
        lastBState = gamepad1.b;
        
        double currentAngle = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
        double power = 0;
        double estVelocity = 0;
        
        if (gamepad1.a) {
            // MPC active
            if (useRealMpc) {
                power = realMpc.update(currentAngle, targetAngle);
                estVelocity = realMpc.getEstimatedVelocity();
            } else {
                power = simpleMpc.update(currentAngle, targetAngle);
            }
            turretMotor.setPower(power);
        } else {
            turretMotor.setPower(0);
            simpleMpc.reset();
            realMpc.reset();
        }
        
        // Safety limits
        int currentPos = turretMotor.getCurrentPosition();
        if (currentPos > TURRET_MAX_TICKS || currentPos < TURRET_MIN_TICKS) {
            turretMotor.setPower(0);
            simpleMpc.reset();
            realMpc.reset();
        }
        
        // Telemetry
        telemetry.addData("=== MPC DEBUG ===", "");
        telemetry.addData("Mode", useRealMpc ? "REAL MPC (QP Solver)" : "Simple MPC (Grid Search)");
        telemetry.addData("Loop Time", "%.1f ms", loopTime * 1000);
        telemetry.addData("Target Angle", "%.1f°", targetAngle);
        telemetry.addData("Current Angle", "%.1f°", currentAngle);
        telemetry.addData("Error", "%.1f°", targetAngle - currentAngle);
        telemetry.addData("MPC Power", "%.3f", power);
        if (useRealMpc) {
            telemetry.addData("Est Velocity", "%.1f °/s", estVelocity);
        }
        telemetry.addData("Encoder Ticks", currentPos);
        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("A Button", "Hold to enable MPC");
        telemetry.addData("B Button", "Toggle MPC type");
        telemetry.addData("Left Stick Y", "Set target angle");
        telemetry.addData("Right Bumper", "Reset to 0");
        telemetry.update();
    }
}
