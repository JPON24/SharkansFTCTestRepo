package org.firstinspires.ftc.teamcode.global;

public class constants {


        // Vision / Limelight
        public static final double LIMELIGHT_ANGLE_CONST_D = 21;
        public static final double APRILTAG_HEIGHT_M = 749.3;
        public static final double LIMELIGHT_DIST_BOTTOM_M = 436.42;
        public static final double LIMELIGHT_DIST_CONST_M = APRILTAG_HEIGHT_M - LIMELIGHT_DIST_BOTTOM_M;
        public static final int LIMELIGHT_POLLING_RATE = 50;
        public static final double LIMELIGHT_OFFSET_CONST = 9;
        public static final double LIMELIGHT_DIST_CONST_SHOOTER = 3.0;

        // Turret Hardware (AutoShooter / Standard)
        public static final double TURRET_TICKS_PER_REV = 145.1;
        public static final double TURRET_GEAR_RATIO = 5.0;
        public static final double TURRET_TICKS_PER_DEGREE = (TURRET_TICKS_PER_REV * TURRET_GEAR_RATIO) / 360.0;

        // Turret Limits (Degrees converted to Ticks)
        public static final double TURRET_MAX_DEG = 90; // 235
        public static final double TURRET_MIN_DEG = -90; // -115
        public static final int TURRET_MAX_TICKS = (int)(TURRET_MAX_DEG * TURRET_TICKS_PER_DEGREE);
        public static final int TURRET_MIN_TICKS = (int)(TURRET_MIN_DEG * TURRET_TICKS_PER_DEGREE);

        // Turret PID & Control
        public static final double TURRET_KP = 0.1; // 0.01
        public static final double TURRET_KI = 0.0; // 0.0001
        public static final double TURRET_KD = 0.0; // 0.001
        public static final double TURRET_KF = 0.0;
        public static final double TURRET_ADAPTIVE_KP_HIGH = 0.008;
        public static final double TURRET_ADAPTIVE_KP_MED = 0.004;
        public static final double TURRET_FILTER_STRENGTH = 0.8;
        public static final double TURRET_DEADBAND = 3.0; // From AutoShooter (was 4.5 in prev batch, reverting to 3 based on this file)
        public static final double TURRET_MAX_POWER = 0.7;
        public static final double TURRET_MAX_DELTA_POWER = 0.03;
        public static final double TURRET_MIN_SPEED = 0.1;
        public static final double TURRET_MANUAL_SPEED = 120; // deg/s
        public static final double TURRET_AUTO_SPEED = 0.4;
        public static final double TURRET_LENIENCE = 5;

        // Shooter / Flywheel
        public static final double SHOOTER_COUNTS_PER_MOTOR_REV = 28.0;
        public static final double SHOOTER_GEAR_REDUCTION = 1;
        public static final double SHOOTER_COUNTS_PER_WHEEL_REV = SHOOTER_COUNTS_PER_MOTOR_REV * SHOOTER_GEAR_REDUCTION;
        public static final double SHOOTER_MAX_RPM = 6000;
        public static final double SHOOTER_MIN_RPM = 0;
        public static final double SHOOTER_RPM_INCREMENT = 100;
        public static final double SHOOTER_RPM_INCREMENT_SMALL = 50;
        public static final double SHOOTER_RPM_LENIENCE = 150;
        public static final double BANGBANG_COEF = 1.2;

        // Shooter PIDF
        public static final double SHOOTER_PID_NORMAL_P = 100;
        public static final double SHOOTER_PID_BOOST_P = 200;
        public static final double SHOOTER_RPM_DROP_THRESHOLD = 300;

        // AutoShooter Preset RPMs
        public static final double SHOOTER_PRESET_HIGH_RPM = 3300;

        // Hood Servos
        public static final double HOOD_INCREMENT = 0.05;
        public static final double HOOD_MAX_POS = 0.65;
        public static final double HOOD_MIN_POS = 0;

        // Hood Presets
        public static final double HOOD_POS_FAR_LOB = 0.40;
        public static final double HOOD_POS_FAR_HARD = 0.2; // Updated from AutoShooter
        public static final double HOOD_POS_MEDIUM = 0.30;
        public static final double HOOD_POS_CLOSE = 0.40;
        public static final double HOOD_POS_DEFAULT = 0.2; // Updated from AutoShooter

        // Intake Subsystem
        public static final double INTAKE_SPEED = 1.0;
        public static final double INTAKE_TRANSFER_SPEED_INTAKE = -0.16;
        public static final double INTAKE_TRANSFER_SPEED_OUTTAKE = -1.0;

        // Swerve Drive (Subsystem)
        public static final double CHASSIS_LENGTH = 0.98;
        public static final double CHASSIS_WIDTH = 1.0;

        // Deadspot Configuration
        public static final double SWERVE_DEADSPOT_START = 315.0;
        public static final double SWERVE_DEADSPOT_END_ENTER = 20.0;
        public static final double SWERVE_DEADSPOT_END_EXIT = 15.0;
        public static final double SWERVE_DEADSPOT_BOUNDARY_MARGIN = 5.0;

        // Swerve PID (Subsystem)
        public static final double SWERVE_PID_P = 0.004;
        public static final double SWERVE_PID_I = 0.0001;
        public static final double SWERVE_PID_D = 0.0003;

        public static final double SWERVE_MIN_SERVO_POWER = 0.03;
        public static final double SWERVE_ANGLE_HOLD_SPEED = 0.05;

        // Swerve Offsets (Subsystem)
        public static final double SWERVE_FL_OFFSET = 0.5;
        public static final double SWERVE_FR_OFFSET = 0.5;
        public static final double SWERVE_BL_OFFSET = 0.5;
        public static final double SWERVE_BR_OFFSET = 0.5;

        public static final double SWERVE_SERVO_JITTER_THRESHOLD = 0.00396825396;
        public static final double SWERVE_UPDATE_HZ = 6;
        public static final double SWERVE_DELTA_MAX = 25;
        public static final double SWERVE_MAX_SPEED = 0.95;
        public static final double SWERVE_DEADZONE = 0.05;

        // SharkDrive & Odometry
        // PID Coefficients
        public static final double SHARK_KPX = 0.1;
        public static final double SHARK_KPY = 0.1;
        public static final double SHARK_KDX = 0;
        public static final double SHARK_KDY = 0;
        public static final double SHARK_KPH = 0;
        public static final double SHARK_KIH = 0;
        public static final double SHARK_KDH = 0;

        // Dih control
        public static final double SHARK_DIH_P = 0.1;
        public static final double SHARK_DIH_I = 0;
        public static final double SHARK_DIH_D = 0;
        public static final double SHARK_DIH_BAND = 5;

        public static final double SHARK_MAX_OUTPUT = 1.0;
        public static final double SHARK_ODO_MIX_PERCENT = 0.7;
        public static final double SHARK_ANGLE_LENIENCE_DEFAULT = 60;
        public static final double SHARK_ANGLE_LENIENCE_STRICT = 15;

        public static final double SHARK_AUTOGRAB_ZERO_X = 10;
        public static final double SHARK_AUTOGRAB_ZERO_Y = 0;

        // Autonomous Constants
        public static final double AUTON_SPEED_STANDARD = 0.7;
        public static final double AUTON_SPEED_LEAVE = 0.811;
        public static final int AUTON_GATE_INTAKE_MS = 2000;
        public static final int AUTON_ROTATE_DURATION_MS = 600;
        public static final int AUTON_FORWARD_DURATION_MS = 1000;
        public static final double AUTON_TURN_POWER = 0.5;

        // Distance Limits for Auto Shooting Logic
        public static final double DIST_LIMIT_CLOSE = 40.0;
        public static final double DIST_LIMIT_MEDIUM = 55.0;

        // Filter Constants
        public static final double DEFAULT_FILTER_ALPHA = 0.3;


        // Zephyr hardware differs slightly
        public static final double ZEPHYR_TICKS_PER_REV = 28.0;
        public static final double ZEPHYR_GEAR_RATIO = 5.0;
        public static final double ZEPHYR_TICKS_PER_DEGREE = (ZEPHYR_TICKS_PER_REV * ZEPHYR_GEAR_RATIO) / 360.0;
        public static final double ZEPHYR_TURRET_PID_P = 0.005;
        public static final double ZEPHYR_TURRET_PID_I = 0.0001;
        public static final double ZEPHYR_TURRET_PID_D = 0.0002;

        // Testing / Legacy Offsets (TestMode, SwerveTest)
        public static final double TEST_FL_OFFSET = 0.22;
        public static final double TEST_FR_OFFSET = 0.42;
        public static final double TEST_BL_OFFSET = 0.16;
        public static final double TEST_BR_OFFSET = 0.5;

        public static final double SWERVE_TEST_FL_OFFSET = 10.0;
        public static final double SWERVE_TEST_KP = 0.005;

        // Hardware Wrapper Thresholds
        public static final double MOTOR_POWER_THRESHOLD = 0.001;
        public static final double MOTOREX_POWER_THRESHOLD = 0.005;
        public static final double SERVO_POSITION_THRESHOLD = 0.001;
        public static final double CRSERVO_POWER_THRESHOLD = 0.01;
        public static final double CRSERVO_KALMAN_Q = 0.1;
        public static final double CRSERVO_KALMAN_R = 3.0;
        public static final double ANALOG_VOLTAGE_REF = 3.3;

        // Voltage Compensation
        public static final double VOLTAGE_KALMAN_Q = 0.001;
        public static final double VOLTAGE_KALMAN_R = 0.5;
        public static final double VOLTAGE_BASELINE_DEFAULT = 11.5;
        public static final double VOLTAGE_CACHED_DEFAULT = 12.0;
        public static final double VOLTAGE_SAFETY_MIN = 5.0;
        public static final int VOLTAGE_SAMPLE_COUNT = 10;

        // Mecanum Drivetrain
        public static final double MECANUM_STRAFE_FACTOR = 1.1;

        // VirtualGoalShooter
        public static final double VGS_WHEEL_RADIUS = 2.83465 / 2;
        public static final double VGS_GEAR_RATIO = 1.0;
        public static final double VGS_HOOD_MIN_ANGLE = 0.0;
        public static final double VGS_HOOD_MAX_ANGLE = 60.0;
        public static final double VGS_TURRET_MAX_INTEGRAL = 25.0;
        public static final int VGS_UNWIND_COOLDOWN = 50;
        public static final double VGS_SOFT_LIMIT_ZONE = 15.0;
        public static final double VGS_WALL_PROTECTION_ZONE = 3.0;
        public static final double VGS_TURRET_POWER_SCALE = 0.2;

        public static final double VIRT_BLUE_BASKET_X = 2.5;
        public static final double VIRT_BLUE_BASKET_Y = 7.5;
        public static final double VIRT_RED_BASKET_X = -2.5;
        public static final double VIRT_RED_BASKET_Y = 7.5;
        public static final double VIRT_RED_BASKET_FAR_X = -110;
        public static final double VIRT_RED_BASKET_FAR_Y = 35;
        public static final double VIRT_BLUE_BASKET_FAR_X = 110;
        public static final double VIRT_BLUE_BASKET_FAR_Y = 35;

        public static final double VIRT_BLUE_HUMAN_ZONE_X = 0.0;
        public static final double VIRT_BLUE_HUMAN_ZONE_Y = 100.0;

        public static final double VGS_TURRET_KALMAN_Q = 0.2; // To disable filter set Q to 1 and R to 0
        public static final double VGS_TURRET_KALMAN_R = 0.01;

        // AutoPIDTuner
        public static final int TUNER_CROSSINGS_CONFIRM = 6;
        public static final double TUNER_STABILITY_THRESHOLD = 0.15;
        public static final int TUNER_PERIODS_TO_MEASURE = 4;
        public static final double TUNER_KP_MIN_DEFAULT = 0.001;
        public static final double TUNER_KP_MAX_DEFAULT = 0.1;
        public static final double TUNER_KP_STEP_DEFAULT = 0.001;
        public static final double TUNER_MAX_POWER_DEFAULT = 0.5;
        public static final double TUNER_STEP_TIMEOUT_DEFAULT = 8.0;
        public static final double TUNER_ZN_KP_MULT = 0.6;
        public static final double TUNER_ZN_KI_MULT = 1.2;
        public static final double TUNER_ZN_KD_MULT = 0.075;
        public static final double TUNER_ZERO_CROSS_MIN_ERROR = 0.5;
}
