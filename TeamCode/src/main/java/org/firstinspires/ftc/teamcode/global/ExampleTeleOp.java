package org.firstinspires.ftc.teamcode.global;

/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║                   SHARKANS GLOBAL LIBRARY                       ║
 * ║                   Example TeleOp Reference                      ║
 * ║                                                                 ║
 * ║  This file demonstrates EVERY component in the global library.  ║
 * ║  Use it as a reference when building your own OpModes.          ║
 * ╚══════════════════════════════════════════════════════════════════╝
 *
 * TABLE OF CONTENTS:
 * ──────────────────
 * 1. HardwareUtil     — Voltage compensation & bulk reads
 * 2. MotorEx          — PID-controlled motor with voltage comp
 * 3. Motor            — Simple cached motor
 * 4. ServoEx          — Cached positional servo
 * 5. CRServoEx        — PID-controlled CR servo with analog encoder
 * 6. PIDController    — PIDF controller with anti-windup
 * 7. KalmanFilter     — Noise-filtering sensor values
 * 8. AnalogFilter     — Low-pass filter for analog encoders
 * 9. EnhancedGamepad  — Button press/release/toggle detection
 * 10. InterpLUT       — Interpolating lookup table for tuning curves
 * 11. LinearMath      — Angle wrapping, clamping, lerp
 * 12. Vector2D        — 2D vector math
 * 13. MecanumDrive    — Field-centric mecanum drivetrain
 * 14. CoaxialSwerve   — 4-module coaxial swerve drivetrain
 */

// ─── FTC SDK Imports ───
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// ─── Hardware Wrappers ───
import org.firstinspires.ftc.teamcode.global.hardware.HardwareUtil;
import org.firstinspires.ftc.teamcode.global.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.global.hardware.Motor;
import org.firstinspires.ftc.teamcode.global.hardware.ServoEx;
import org.firstinspires.ftc.teamcode.global.hardware.CRServoEx;

// ─── Control Systems ───
import org.firstinspires.ftc.teamcode.global.control.PIDController;
import org.firstinspires.ftc.teamcode.global.control.KalmanFilter;
import org.firstinspires.ftc.teamcode.global.control.AnalogFilter;

// ─── Drivetrains ───
import org.firstinspires.ftc.teamcode.global.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.global.drivetrain.CoaxialSwerve;

// ─── Utilities ───
import org.firstinspires.ftc.teamcode.global.util.gamepad.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.global.util.math.InterpLUT;
import org.firstinspires.ftc.teamcode.global.util.math.LinearMath;
import org.firstinspires.ftc.teamcode.global.util.math.Vector2D;


@TeleOp(name = "⭐ Example TeleOp", group = "Examples")
public class ExampleTeleOp extends OpMode {

    // ╔══════════════════════════════════════════════════════════════╗
    // ║ SECTION 1: DECLARE YOUR OBJECTS                             ║
    // ║                                                             ║
    // ║ Declare everything here so it's visible to both init()      ║
    // ║ and loop(). Initialize them inside init().                  ║
    // ╚══════════════════════════════════════════════════════════════╝

    // ── 1. HardwareUtil ──
    // Manages: bulk reads, voltage monitoring, voltage compensation
    // Create ONE instance and pass it to all hardware that needs it.
    HardwareUtil hardwareUtil;

    // ── 2. MotorEx ──
    // A DC motor with: PID position control + power caching + voltage compensation
    // Use for: arms, slides, turrets — anything that needs to go to specific positions
    MotorEx armMotor;

    // ── 3. Motor ──
    // A simple DC motor with: power caching (+ optional voltage compensation)
    // Use for: intakes, flywheels — anything that just needs raw power
    Motor intakeMotor;

    // ── 4. ServoEx ──
    // A positional servo with: position caching + input clamping
    // Use for: claws, grippers, hoods — anything that goes to set positions
    ServoEx clawServo;

    // ── 5. CRServoEx ──
    // A continuous rotation servo with: PID + analog encoder + filtering + voltage comp
    // Use for: swerve steering, turrets — anything that needs closed-loop angle control
    // (Normally you won't use these standalone — CoaxialSwerve handles them internally)
    CRServoEx exampleCRServo;

    // ── 6. PIDController ──
    // A PIDF controller with anti-windup and angle-safe error handling
    // Use for: custom control loops (turret tracking, heading hold, etc.)
    PIDController headingPID;

    // ── 7. KalmanFilter ──
    // Filters noisy sensor values using a 1D Kalman filter
    // Use for: smoothing distance sensors, voltage readings, encoder velocity
    KalmanFilter distanceFilter;

    // ── 8. AnalogFilter ──
    // Low-pass filter designed for analog encoders with angle wrap-around handling
    // Use for: smoothing analog encoder readings (used internally by CRServoEx)
    AnalogFilter analogFilter;

    // ── 9. EnhancedGamepad ──
    // Wraps the gamepad with press/release/toggle detection for every button
    // Use for: toggling states, detecting single presses, handling triggers
    EnhancedGamepad gp1;
    EnhancedGamepad gp2;

    // ── 10. InterpLUT ──
    // Interpolating lookup table — give it data points, it interpolates between them
    // Use for: RPM curves, hood angles, distance-to-power mappings
    InterpLUT speedCurve;

    // ── 11-12. LinearMath & Vector2D ──
    // Static math utilities — no objects to declare, just call the methods directly
    // LinearMath: angleWrap(), clamp(), lerp(), map(), epsilonEquals()
    // Vector2D: immutable 2D vectors with add, subtract, rotate, magnitude, angle

    // ── 13-14. Drivetrains ──
    // Pick ONE drivetrain for your robot. Both are shown here for reference.
    MecanumDrive mecanumDrive;
    CoaxialSwerve swerveDrive;

    // ── Sensors ──
    SparkFunOTOS otos;  // Used for heading in field-centric driving

    // ── State Variables ──
    boolean usingSwerve = true;     // Toggle between drivetrains with a button
    boolean clawOpen = false;       // Track claw state for toggle
    double armTargetPosition = 0;   // Target encoder position for the arm


    // ╔══════════════════════════════════════════════════════════════╗
    // ║ SECTION 2: INIT — Runs ONCE when you press INIT             ║
    // ║                                                             ║
    // ║ Create and configure all your objects here.                 ║
    // ║ ORDER MATTERS: HardwareUtil first, then everything else.    ║
    // ╚══════════════════════════════════════════════════════════════╝

    @Override
    public void init() {

        // ────────────────────────────────────────────
        // STEP 1: HardwareUtil (ALWAYS FIRST!)
        // ────────────────────────────────────────────
        // This sets up:
        //   • Bulk read mode (faster sensor reads)
        //   • Voltage sensor + Kalman filter for compensation
        //
        // Pass this to all motors/servos that need voltage compensation.

        hardwareUtil = new HardwareUtil(hardwareMap);

        telemetry.addData("✅ HardwareUtil", "Baseline voltage: %.2fV", hardwareUtil.getBaseline());


        // ────────────────────────────────────────────
        // STEP 2: Motors
        // ────────────────────────────────────────────

        // MotorEx: For precise position control (arm, slide, turret)
        //   Args: hardwareMap, hardwareUtil, configName, kP, kI, kD, kF
        //   • kP: Main tuning value (start with 0.01, increase until it responds)
        //   • kI: Usually 0 (avoids oscillation)
        //   • kD: Dampening (start with 0, add small amounts to reduce overshoot)
        //   • kF: Feedforward (usually 0 unless fighting gravity)

        armMotor = new MotorEx(hardwareMap, hardwareUtil, "armMotor", 0.005, 0, 0.0001, 0);

        // Motor: For simple power control (intake, flywheel)
        //   Args: hardwareMap, configName
        //   Optional: hardwareMap, hardwareUtil, configName (for voltage comp)

        intakeMotor = new Motor(hardwareMap, hardwareUtil, "intakeMotor");
        // intakeMotor = new Motor(hardwareMap, "intakeMotor");  // ← Without voltage comp


        // ────────────────────────────────────────────
        // STEP 3: Servos
        // ────────────────────────────────────────────

        // ServoEx: Positional servo with caching
        //   Args: hardwareMap, configName

        clawServo = new ServoEx(hardwareMap, "clawServo");
        clawServo.setPosition(0.5);  // Start at middle position

        // CRServoEx: PID-controlled CR servo with analog encoder
        //   Args: hardwareMap, hardwareUtil, servoName, encoderName,
        //         kP, kI, kD, kF, filterAlpha, offsetDegrees
        //
        //   • filterAlpha: 0.0-1.0 (lower = more smoothing, higher = more responsive)
        //   • offset: Your encoder's zero-point offset in degrees

        exampleCRServo = new CRServoEx(
                hardwareMap,
                hardwareUtil,
                "crServo",        // Servo config name
                "crEncoder",      // Analog encoder config name
                0.015,            // kP (start here for CR servos)
                0,                // kI
                0.0003,           // kD
                0,                // kF
                0.3,              // Filter alpha
                -12.0             // Encoder offset (degrees)
        );


        // ────────────────────────────────────────────
        // STEP 4: Control Systems
        // ────────────────────────────────────────────

        // PIDController: For custom control loops
        //   Args: kP, kI, kD, kF

        headingPID = new PIDController(0.02, 0, 0.001, 0);
        headingPID.setMaxIntegral(1.0);  // Prevent integral windup

        // KalmanFilter: For smoothing noisy sensors
        //   Args: Q (process noise), R (measurement noise)
        //   • Q small (0.001) = trusts the model (slow to change)
        //   • R large (0.1-1.0) = sensor is noisy
        //   Rule of thumb:  Q << R  for smooth output

        distanceFilter = new KalmanFilter(0.001, 0.1);

        // AnalogFilter: Low-pass filter for angle sensors
        //   Args: alpha (0.0-1.0)
        //   Lower alpha = more smoothing (good for noisy encoders)
        //   This is used internally by CRServoEx, shown here for reference

        analogFilter = new AnalogFilter(0.3);


        // ────────────────────────────────────────────
        // STEP 5: Gamepad
        // ────────────────────────────────────────────

        // EnhancedGamepad: Wraps the raw gamepad with nice features
        //   Args: the raw gamepad from the FTC SDK

        gp1 = new EnhancedGamepad(gamepad1);
        gp2 = new EnhancedGamepad(gamepad2);


        // ────────────────────────────────────────────
        // STEP 6: InterpLUT (Lookup Table)
        // ────────────────────────────────────────────

        // Add data points: input → output
        // The table will interpolate between points automatically.
        //
        // Example: distance (inches) → motor speed
        //   At 10 inches: speed 0.3
        //   At 30 inches: speed 0.6
        //   At 20 inches: speed will be ~0.45 (interpolated!)

        speedCurve = new InterpLUT("speed");
        speedCurve.add(10, 0.3);   // Close = slow
        speedCurve.add(30, 0.6);   // Medium = medium
        speedCurve.add(60, 0.9);   // Far = fast
        speedCurve.add(100, 1.0);  // Very far = full speed


        // ────────────────────────────────────────────
        // STEP 7: OTOS (for heading)
        // ────────────────────────────────────────────

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(new SparkFunOTOS.Pose2D(0, -3.74016, 0));
        otos.calibrateImu();


        // ────────────────────────────────────────────
        // STEP 8: Drivetrains (pick ONE for your robot)
        // ────────────────────────────────────────────

        // ── OPTION A: Mecanum Drive ──
        // Simple 4-motor mecanum with field-centric driving

        mecanumDrive = new MecanumDrive();
        mecanumDrive.mecanumConstants(
                hardwareMap,
                "frontLeft",    // Motor config names
                "frontRight",
                "backLeft",
                "backRight"
        );
        mecanumDrive.setScalar(0.8);  // 80% max speed


        // ── OPTION B: Coaxial Swerve Drive ──
        // 4-module swerve with CR servos + analog encoders + PID steering

        swerveDrive = new CoaxialSwerve();

        swerveDrive.initFrontLeft(hardwareMap, hardwareUtil,
                "flServo", "flMotor", "flEncoder",
                0.015, 0, 0.0003, 0, 0.3, -12);

        swerveDrive.initFrontRight(hardwareMap, hardwareUtil,
                "frServo", "frMotor", "frEncoder",
                0.015, 0, 0.0003, 0, 0.3, -45);

        swerveDrive.initBackLeft(hardwareMap, hardwareUtil,
                "blServo", "blMotor", "blEncoder",
                0.015, 0, 0.0003, 0, 0.3, 30);

        swerveDrive.initBackRight(hardwareMap, hardwareUtil,
                "brServo", "brMotor", "brEncoder",
                0.015, 0, 0.0003, 0, 0.3, 10);

        // Robot dimensions: width (in), length (in), height (in), max speed scalar
        swerveDrive.CoaxialSwerveConstants(11, 10, 9, 0.8);

        // IMPORTANT: Call initialize() LAST — reads current encoder angles to prevent
        // the servos from spinning to 0 on startup
        swerveDrive.initialize();


        telemetry.addData("✅ Init", "Complete! Press Play to start.");
        telemetry.update();
    }


    // ╔══════════════════════════════════════════════════════════════╗
    // ║ SECTION 3: LOOP — Runs 50+ times per second                 ║
    // ║                                                             ║
    // ║ This is your main control loop. Keep it fast!               ║
    // ║ The order of operations matters:                            ║
    // ║   1. Update HardwareUtil (clears bulk cache)                ║
    // ║   2. Update gamepads (reads button states)                  ║
    // ║   3. Read sensors                                           ║
    // ║   4. Do your logic                                          ║
    // ║   5. Set outputs (motors, servos)                           ║
    // ║   6. Telemetry                                              ║
    // ╚══════════════════════════════════════════════════════════════╝

    @Override
    public void loop() {

        // ────────────────────────────────────────────
        // 1. UPDATE HardwareUtil (ALWAYS FIRST IN LOOP!)
        // ────────────────────────────────────────────
        // • Clears the bulk read cache so sensor values are fresh
        // • Updates the filtered voltage for compensation

        hardwareUtil.update();


        // ────────────────────────────────────────────
        // 2. UPDATE GAMEPADS
        // ────────────────────────────────────────────
        // Must call update() every loop BEFORE reading button states

        gp1.update();
        gp2.update();


        // ────────────────────────────────────────────
        // 3. READ SENSORS
        // ────────────────────────────────────────────

        double heading = otos.getPosition().h;


        // ────────────────────────────────────────────
        // 4. DRIVING (choose your drivetrain)
        // ────────────────────────────────────────────

        // Toggle drivetrain with the back button
        if (gp1.back.wasJustPressed()) {
            usingSwerve = !usingSwerve;
        }

        // Read joystick inputs
        double leftX  = gp1.left_stick_x;   // Strafe
        double leftY  = -gp1.left_stick_y;  // Forward (negative because Y is inverted)
        double rightX = gp1.right_stick_x;  // Turn

        if (usingSwerve) {
            // Swerve: pass x, y, turn, and heading for field-centric
            swerveDrive.drive(leftX, leftY, rightX, heading);
        } else {
            // Mecanum: same inputs, heading in radians
            mecanumDrive.drive(leftX, leftY, rightX, Math.toRadians(heading));
        }


        // ────────────────────────────────────────────
        // 5. ARM CONTROL (MotorEx — PID position)
        // ────────────────────────────────────────────
        // D-pad presets or manual control with right stick

        if (gp2.dpad_up.wasJustPressed()) {
            armTargetPosition = 500;   // High position (encoder ticks)
        }
        if (gp2.dpad_down.wasJustPressed()) {
            armTargetPosition = 0;     // Home position
        }
        if (gp2.dpad_left.wasJustPressed()) {
            armTargetPosition = 250;   // Mid position
        }

        // Fine adjustment with right stick
        armTargetPosition += gp2.right_stick_y * -5;  // 5 ticks per loop

        // This runs the PID loop internally and moves the motor
        armMotor.setPosition(armTargetPosition);


        // ────────────────────────────────────────────
        // 6. INTAKE (Motor — raw power)
        // ────────────────────────────────────────────

        if (gp2.right_bumper.isPressed) {
            intakeMotor.setPower(1.0);         // Intake
        } else if (gp2.left_bumper.isPressed) {
            intakeMotor.setPower(-1.0);        // Outtake
        } else {
            intakeMotor.setPower(0);           // Stop
        }


        // ────────────────────────────────────────────
        // 7. CLAW (ServoEx — toggle with a button)
        // ────────────────────────────────────────────

        // Toggle claw open/closed with A button (single press, not hold)
        if (gp2.a.wasJustPressed()) {
            clawOpen = !clawOpen;
        }
        clawServo.setPosition(clawOpen ? 0.8 : 0.2);


        // ────────────────────────────────────────────
        // 8. CR SERVO (CRServoEx — PID angle control)
        // ────────────────────────────────────────────
        // Set a target angle, then call update() every loop

        if (gp2.x.wasJustPressed()) {
            exampleCRServo.setPosition(90);    // Turn to 90°
        }
        if (gp2.b.wasJustPressed()) {
            exampleCRServo.setPosition(-90);   // Turn to -90°
        }
        if (gp2.y.wasJustPressed()) {
            exampleCRServo.setPosition(0);     // Turn to 0°
        }

        // CRITICAL: Must call update() every loop to run the PID!
        exampleCRServo.update();


        // ────────────────────────────────────────────
        // 9. LINEARMATH EXAMPLES
        // ────────────────────────────────────────────

        // angleWrap: Keeps angles in -180 to 180 range
        double wrappedAngle = LinearMath.angleWrap(heading);

        // clamp: Keeps a value within bounds
        double clampedPower = LinearMath.clamp(leftY * 2.0, -1.0, 1.0);

        // lerp: Smooth interpolation between two values
        // lerp(start, end, t) where t is 0.0 to 1.0
        double smoothValue = LinearMath.lerp(0, 100, 0.5);  // = 50

        // map: Remap a value from one range to another
        // map(value, fromMin, fromMax, toMin, toMax)
        double mappedTrigger = LinearMath.map(gp1.right_trigger, 0, 1, 0.3, 1.0);


        // ────────────────────────────────────────────
        // 10. VECTOR2D EXAMPLES
        // ────────────────────────────────────────────

        Vector2D robotPos = new Vector2D(10, 20);
        Vector2D targetPos = new Vector2D(50, 30);

        // Subtract to get direction vector
        Vector2D direction = targetPos.subtract(robotPos);

        // Get distance and angle to target
        double distanceToTarget = direction.magnitude();
        double angleToTarget = direction.angle();  // in degrees

        // Rotate a vector by an angle (useful for field-centric math)
        Vector2D rotated = robotPos.rotate(45);  // Rotate 45 degrees


        // ────────────────────────────────────────────
        // 11. INTERPLUT EXAMPLE
        // ────────────────────────────────────────────

        // Get interpolated speed for a given distance
        // If distance is 20 → interpolates between the 10→0.3 and 30→0.6 entries
        double recommendedSpeed = speedCurve.get(distanceToTarget);


        // ────────────────────────────────────────────
        // 12. KALMAN FILTER EXAMPLE
        // ────────────────────────────────────────────

        // Feed it raw noisy readings, get clean output
        // double rawDistance = distanceSensor.getDistance(DistanceUnit.INCH);
        // double cleanDistance = distanceFilter.filter(rawDistance);


        // ────────────────────────────────────────────
        // 13. PID CONTROLLER EXAMPLE (manual usage)
        // ────────────────────────────────────────────

        // Example: Hold a specific heading using PID
        // double headingTarget = 90;
        // double headingError = LinearMath.angleWrap(headingTarget - heading);
        // double turnCorrection = headingPID.updateWithError(headingError, headingTarget);
        // ^ Use this to replace or augment the driver's turn input


        // ────────────────────────────────────────────
        // TELEMETRY — Show useful debug info
        // ────────────────────────────────────────────

        telemetry.addLine("═══ DRIVETRAIN ═══");
        telemetry.addData("Mode", usingSwerve ? "🔄 Swerve" : "🔀 Mecanum");
        telemetry.addData("Heading", "%.1f°", heading);
        telemetry.addData("Stick", "(%.2f, %.2f) turn=%.2f", leftX, leftY, rightX);

        telemetry.addLine("\n═══ ARM ═══");
        telemetry.addData("Target", "%.0f ticks", armTargetPosition);
        telemetry.addData("Current", "%d ticks", armMotor.getPosition());

        telemetry.addLine("\n═══ CLAW ═══");
        telemetry.addData("State", clawOpen ? "🟢 OPEN" : "🔴 CLOSED");

        telemetry.addLine("\n═══ CR SERVO ═══");
        telemetry.addData("Target", "%.1f°", exampleCRServo.getTargetAngle());
        telemetry.addData("Current", "%.1f°", exampleCRServo.getAngle());
        telemetry.addData("Error", "%.1f°", exampleCRServo.getError());

        telemetry.addLine("\n═══ HARDWARE ═══");
        telemetry.addData("Voltage", "%.2fV (baseline: %.2fV)",
                hardwareUtil.getVoltage(), hardwareUtil.getBaseline());
        telemetry.addData("Voltage Multiplier", "%.3fx", hardwareUtil.getVoltageMultiplier());

        telemetry.addLine("\n═══ MATH DEMO ═══");
        telemetry.addData("Distance to target", "%.1f in", distanceToTarget);
        telemetry.addData("Angle to target", "%.1f°", angleToTarget);
        telemetry.addData("InterpLUT speed", "%.2f", recommendedSpeed);

        telemetry.update();
    }
}
