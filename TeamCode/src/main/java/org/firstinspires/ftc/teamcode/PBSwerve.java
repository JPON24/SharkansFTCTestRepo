//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//
//@TeleOp
//public class PBSwerve extends OpMode{
//
//    private ShooterSubsystem shooter;
//    private floatingIntake intake;
//
//    private SparkFunOTOS otos;
//
//    private final double L = 0.98;
//    private final double W = 1.00;
//
//    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
//    private CRServo frontLeftServo, frontRightServo, backLeftServo, backRightServo;
//    private AnalogInput frontLeftAnalog, frontRightAnalog, backLeftAnalog, backRightAnalog;
//
//
//    private boolean lastRightBumperState = false;
//    private boolean lastLeftBumperState = false;
//
//    private PIDController flPID, frPID, rlPID, rrPID;
//    private ElapsedTime pidTimer = new ElapsedTime();
//
//    private double FLkP = 0.003, FLkI = 0.0001, FLkD = 0.0001;
//    private double FRkP = 0.003, FRkI = 0.0001, FRkD = 0.0001;
//    private double BLkP = 0.003, BLkI = 0.0001, BLkD = 0.0001;
//    private double BRkP = 0.003, BRkI = 0.0001, BRkD = 0.0001;
//    private double minServoPower = 0.03;
//    private double ANGLE_HOLD_SPEED = 0.05;
//
//    private double FL_OFFSET = -83;
//    private double FR_OFFSET = 151;
//    private double BL_OFFSET = 69;
//    private double BR_OFFSET = -20;
//
//    private double speed = 0.75;
//    private double lastTargetFL = 0, lastTargetFR = 0, lastTargetRL = 0, lastTargetRR = 0;
//    private double flSpeed, frSpeed, blSpeed, brSpeed;
//    private double angleFL, angleFR, angleRL, angleRR;
//
//
//    @Override
//    public void init() {
//
//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
//        otos.setOffset(new SparkFunOTOS.Pose2D(0, -3.74016, 0));
//        otos.calibrateImu();
//
//        // Initialize subsystems
//        shooter = new ShooterSubsystem();
//        shooter.init(hardwareMap, otos);
//
//        intake = new floatingIntake();
//        intake.init(hardwareMap);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//
//
//
//        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor"); // Ctrl h 3 None of these are correct...   ```
//        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor"); // Exp h 1
//        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor"); // Ctrl h 0
//        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor"); // Exp h 2
//
//        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo"); // Ctrl h 1
//        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo"); // Exp h 5
//        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo"); // Ctrl h 2
//        backRightServo = hardwareMap.get(CRServo.class, "backRightServo"); // Exp h 4
//
//        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog"); // Ctrl h 1
//        frontRightAnalog = hardwareMap.get(AnalogInput.class, "frontRightAnalog"); // Exp h 1
//        backLeftAnalog = hardwareMap.get(AnalogInput.class, "backLeftAnalog"); // Ctrl h 2
//        backRightAnalog = hardwareMap.get(AnalogInput.class, "backRightAnalog"); // Exp h2
//
//
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
//
//        flPID = new PIDController(FLkP, FLkI, FLkD);
//        frPID = new PIDController(FRkP, FRkI, FRkD);
//        rlPID = new PIDController(BLkP, BLkI, BLkD);
//        rrPID = new PIDController(BRkP, BRkI, BRkD);
//
//        lastTargetFL = getAngle(frontLeftAnalog, FL_OFFSET);
//        lastTargetFR = getAngle(frontRightAnalog, FR_OFFSET);
//        lastTargetRL = getAngle(backLeftAnalog, BL_OFFSET);
//        lastTargetRR = getAngle(backRightAnalog, BR_OFFSET);
//
//
//    }
//
//    @Override
//    public void loop() {
//
//        double leftStickX = gamepad1.left_stick_x;
//        double leftStickY = -gamepad1.left_stick_y;
//        double rightStickX = gamepad1.right_stick_x;
//        double driveEnabled = gamepad1.right_trigger;
//
//        drive(leftStickY, leftStickX, rightStickX, driveEnabled);
//
//        boolean shootButtonPressed = gamepad2.a;
//        boolean hardShotPressed = gamepad2.b;
//
//        boolean gamepad1A = gamepad1.a;
//        boolean rightBumperPressed = gamepad2.right_bumper;
//        boolean leftBumperPressed = gamepad2.left_bumper;
//
//        boolean willIncrement = (rightBumperPressed && !lastRightBumperState);
//        boolean willDecrement = (leftBumperPressed && !lastLeftBumperState);
//
//        if (willIncrement) {
//            if (shooter.getTargetRPM() + 100 < 6000) {
//                shooter.setTargetRPM(shooter.getTargetRPM() + 100);
//                shooter.BangBang();
//            }
//        } else if (willDecrement) {
//            if (shooter.getTargetRPM() - 100 >= 0) {
//                shooter.setTargetRPM(shooter.getTargetRPM() - 100);
//            }
//        }
//
////        if (shooter.getCurrentRPM() > 0) {
//////            shooter.setLEDLight();
//        shooter.BangBang();
////        }
//
//        lastRightBumperState = rightBumperPressed;
//        lastLeftBumperState = leftBumperPressed;
//
////        shooter.setTargetRPM(shooter.getTargetRPM());
//
//        shooter.update();
//
//        // Press Y to calibrate and switch to predictive mode
////        if (gamepad1.y) {
////            shooter.requestCalibration();
////        }
//
//        if (gamepad2.right_trigger > 0.5)
//        {
//            shooter.decideManualOrTxBLUE(-1);
//        }
//        else if (gamepad2.left_trigger > 0.5)
//        {
//            shooter.decideManualOrTxBLUE(1);
//        }
//        else {
//            shooter.decideManualOrTxBLUE(0);
//        }
////        shooter.trackTargetHybrid();
//
//        if (gamepad1.dpad_left) {
//            intake.intake(true);
//        } else if (gamepad1.dpad_right) {
//            intake.outtake(true);
//        } else {
//            intake.intake(false);
//            intake.outtake(false);
//        }
//
//        if (gamepad1A) {
//            reZero();
//        }
//
//        telemetry.addData("=== SWERVE ===", "");
//        telemetry.addData("Wheels (FL/FR/BL/BR)", "%.0f° %.0f° %.0f° %.0f°",
//                getAngle(frontLeftAnalog, FL_OFFSET), getAngle(frontRightAnalog, FR_OFFSET),
//                getAngle(backLeftAnalog, BL_OFFSET), getAngle(backRightAnalog, BR_OFFSET));
//        telemetry.addData("RawServoAngle, (FL/FR/BL/BR", "%.0f° %.0f° %.0f° %.0f°",
//                getFLRawAngle(), getFRRawAngle(),
//                getBLRawAngle(), getBRRawAngle());
//        telemetry.update();
//
//
//    }
//
//    public void drive(double y_cmd, double x_cmd, double turn_cmd, double driveEnabled) {
//        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
//            stop();
//            return;
//        }
//
//
////        if (otos != null) {
////            double heading = otos.getPosition().h;  // OTOS outputs radians
////            double cos = Math.cos(-heading);
////            double sin = Math.sin(-heading);
////            double temp = x_cmd * cos - y_cmd * sin;
////            y_cmd = x_cmd * sin + y_cmd * cos;
////            x_cmd = temp;
////        }
//
//        double y_fr = y_cmd + turn_cmd * L;
//        double x_fr = x_cmd - turn_cmd * W;
//
//        double y_fl = y_cmd - turn_cmd * L;
//        double x_fl = x_cmd - turn_cmd * W;
//
//        double y_rl = y_cmd - turn_cmd * L;
//        double x_rl = x_cmd + turn_cmd * W;
//
//        double y_rr = y_cmd + turn_cmd * L;
//        double x_rr = x_cmd + turn_cmd * W;
//
//        double speed_fr = Math.hypot(x_fr, y_fr);
//        double speed_fl = Math.hypot(x_fl, y_fl);
//        double speed_rl = Math.hypot(x_rl, y_rl);
//        double speed_rr = Math.hypot(x_rr, y_rr);
//
//        angleFL = (speed_fl < ANGLE_HOLD_SPEED) ? lastTargetFL
//                : Math.toDegrees(Math.atan2(x_fl, y_fl));
//        angleFR = (speed_fr < ANGLE_HOLD_SPEED) ? lastTargetFR
//                : Math.toDegrees(Math.atan2(x_fr, y_fr));
//        angleRL = (speed_rl < ANGLE_HOLD_SPEED) ? lastTargetRL
//                : Math.toDegrees(Math.atan2(x_rl, y_rl));
//        angleRR = (speed_rr < ANGLE_HOLD_SPEED) ? lastTargetRR
//                : Math.toDegrees(Math.atan2(x_rr, y_rr));
//
//        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
//        if (max > 1.0) {
//            speed_fr /= max;
//            speed_fl /= max;
//            speed_rl /= max;
//            speed_rr /= max;
//        }
//
//        double currentFL = getAngle(frontLeftAnalog, FL_OFFSET);
//        double currentFR = getAngle(frontRightAnalog, FR_OFFSET);
//        double currentRL = getAngle(backLeftAnalog, BL_OFFSET);
//        double currentRR = getAngle(backRightAnalog, BR_OFFSET);
//
//        double[] optFL = optimize(angleFL, speed_fl, currentFL);
//        double[] optFR = optimize(angleFR, speed_fr, currentFR);
//        double[] optRL = optimize(angleRL, speed_rl, currentRL);
//        double[] optRR = optimize(angleRR, speed_rr, currentRR);
//
//        // Motor speed is controlled by right bumper
//        if (driveEnabled > 0.5) {
//            flSpeed = optFL[1] * speed;
//            frSpeed = optFR[1] * speed;
//            blSpeed = optRL[1] * speed;
//            brSpeed = optRR[1] * speed;
//
//            frontLeftMotor.setPower(flSpeed);
//            frontRightMotor.setPower(frSpeed);
//            backLeftMotor.setPower(blSpeed);
//            backRightMotor.setPower(brSpeed);
//        } else {
//            // Motors off when bumper not pressed
//            frontLeftMotor.setPower(0);
//            frontRightMotor.setPower(0);
//            backLeftMotor.setPower(0);
//            backRightMotor.setPower(0);
//            flSpeed = frSpeed = blSpeed = brSpeed = 0;
//        }
//
//        lastTargetFL = optFL[0];
//        lastTargetFR = optFR[0];
//        lastTargetRL = optRL[0];
//        lastTargetRR = optRR[0];
//
//        // Servos always track stick direction (PID runs regardless of bumper)
//        runPID(optFL[0], optFR[0], optRL[0], optRR[0],
//                currentFL, currentFR, currentRL, currentRR);
//    }
//
//    public void stop() {
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        backRightMotor.setPower(0);
//        frontLeftServo.setPower(0);
//        frontRightServo.setPower(0);
//        backLeftServo.setPower(0);
//        backRightServo.setPower(0);
//        flSpeed = frSpeed = blSpeed = brSpeed = 0;
//    }
//
//    public void reZero() {
//        FL_OFFSET = (frontLeftAnalog.getVoltage() / 3.3) * 360.0;
//        FR_OFFSET = (frontRightAnalog.getVoltage() / 3.3) * 360.0;
//        BL_OFFSET = (backLeftAnalog.getVoltage() / 3.3) * 360.0;
//        BR_OFFSET = (backRightAnalog.getVoltage() / 3.3) * 360.0;
//    }
//
//    private void runPID(double targetFL, double targetFR, double targetRL, double targetRR,
//                        double currentFL, double currentFR, double currentRL, double currentRR) {
//        double dt = pidTimer.seconds();
//        if (dt < 0.001) dt = 0.001;
//
//        double powerFL = flPID.calculate(targetFL, currentFL, dt);
//        double powerFR = frPID.calculate(targetFR, currentFR, dt);
//        double powerRL = rlPID.calculate(targetRL, currentRL, dt);
//        double powerRR = rrPID.calculate(targetRR, currentRR, dt);
//
//        frontLeftServo.setPower(powerFL * 1.00);
//        frontRightServo.setPower(powerFR * 1.00);
//        backLeftServo.setPower(powerRL * 1.00);
//        backRightServo.setPower(powerRR * 1.00);
//
//        pidTimer.reset();
//    }
//
//    private double getAngle(AnalogInput sensor, double offset) {
//        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;
//        double adjustedAngle = rawAngle - offset;
//        return normalizeAngle(adjustedAngle);
//    }
//
//    private double normalizeAngle(double angle) {
//        angle = (angle + 180.0) % 360.0;
//        if (angle < 0) angle += 360.0;
//        return angle - 180.0;
//    }
//
//    private double[] optimize(double target, double speed, double current) {
//        double delta = normalizeAngle(target - current);
//        if (Math.abs(delta) > 90) {
//            target = normalizeAngle(target + 180);
//            speed *= -1;
//        }
//        return new double[]{target, speed};
//    }
//
//    // telemetry getters... Brah
//    public double getFLAngle() { return getAngle(frontLeftAnalog, FL_OFFSET); }
//    public double getFRAngle() { return getAngle(frontRightAnalog, FR_OFFSET); }
//    public double getBLAngle() { return getAngle(backLeftAnalog, BL_OFFSET); }
//    public double getBRAngle() { return getAngle(backRightAnalog, BR_OFFSET); }
//
//    public double getFLRawAngle() { return (frontLeftAnalog.getVoltage() / 3.3) * 360.0; }
//    public double getFRRawAngle() { return (frontRightAnalog.getVoltage() / 3.3) * 360.0; }
//    public double getBLRawAngle() { return (backLeftAnalog.getVoltage() / 3.3) * 360.0; }
//    public double getBRRawAngle() { return (backRightAnalog.getVoltage() / 3.3) * 360.0; }
//
//    public class PIDController {
//        private double kP, kI, kD;
//        private double lastError = 0;
//        private double integralSum = 0;
//        private double maxIntegral = 0.5;  // Anti-windup clamp
//
//        public PIDController(double kP, double kI, double kD) {
//            this.kP = kP;
//            this.kI = kI;
//            this.kD = kD;
//        }
//
//        public double calculate(double target, double current, double dt) {
//            double error = normalizeAngle(target - current);
//
//            double pTerm = error * kP;
//
//            integralSum += error * dt;
//            integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);
//            double iTerm = integralSum * kI;
//
//            double derivative = (error - lastError) / dt;
//            double dTerm = derivative * kD;
//
//            lastError = error;
//
//            double output = pTerm + iTerm + dTerm;
//
//            if (Math.abs(error) > 1.50) {
//                output += Math.signum(output) * minServoPower;
//            } else {
//                output = 0;
//                integralSum = 0;  // Reset integral when at target
//            }
//
//            return Range.clip(output, -1.0, 1.0);
//        }
//
//        public void resetIntegral() {
//            integralSum = 0;
//        }
//    }
//}
