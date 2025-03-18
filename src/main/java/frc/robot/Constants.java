package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {

    public static class SwerveModule {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.75;
        public static final double DRIVE_ENCODER_ROTATION_2_METER = (DRIVE_MOTOR_GEAR_RATIO * WHEEL_DIAMETER_METERS
                * Math.PI);
        public static final double DRIVE_ENCODER_RPS_2_MPS = DRIVE_ENCODER_ROTATION_2_METER;
        public static final double TURN_MOTOR_GEAR_RATIO = 1 / 12.8;
        public static final double TURN_ENCODER_ROTATION_2_RAD = (TURN_MOTOR_GEAR_RATIO * 2 * Math.PI);
        public static final double TURN_ENCODER_RPS_2_MPS = TURN_ENCODER_ROTATION_2_RAD;
        public static final double P_TURN = 0.118;
        public static final double D_TURN = 0;
    }

    public static class DriveConstants {
        public static final double PHYSICAL_MAX_SPEED_MPS = 0.3;
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RPM = Math.PI * 0.2;
        public static final double TELE_DRIVE_MAX_ACCELERATION_UPS = 5;
        public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UPS = Math.PI;
        public static final double TELE_DRIVE_MAX_SPEED_MPS = PHYSICAL_MAX_SPEED_MPS / 2;
        public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RPS = PHYSICAL_MAX_ANGULAR_SPEED_RPM / 2;
        public static final int X_AXIS = 0;
        public static final int Y_AXIS = 1;
        public static final int TURN_AXIS = 4;
        public static final int FIELD_ORIENTED_BUTTON = 0;
        public static final double JOYSTICK_DEADBAND = 0.1;
    }

    public static class RobotStructure {
        public static final double TRACK_WIDTH = Units.inchesToMeters(24.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(19);
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                FRONT_LEFT_LOCATION,
                FRONT_RIGHT_LOCATION,
                BACK_LEFT_LOCATION,
                BACK_RIGHT_LOCATION);
    }

    public static class FrontLeft {
        public static final int DRIVE_MOTOR_ID = 11;
        public static final int TURN_MOTOR_ID = 12;
        public static final boolean DRIVE_MOTOR_REVERSED = false;
        public static final boolean TURNING_MOTOR_REVERSED = true;
        public static final int ABSOLUTE_ENCODER_ID = 1;
        public static final double ENCODER_OFFSET = 0.227783203125;
    }

    public static class FrontRight {
        public static final int DRIVE_MOTOR_ID = 41;
        public static final int TURN_MOTOR_ID = 42;
        public static final boolean DRIVE_MOTOR_REVERSED = true;
        public static final boolean TURNING_MOTOR_REVERSED = true;
        public static final int ABSOLUTE_ENCODER_ID = 4;
        public static final double ENCODER_OFFSET = 0.248291015625;
    }

    public static class BackLeft {
        public static final int DRIVE_MOTOR_ID = 21;
        public static final int TURN_MOTOR_ID = 22;
        public static final boolean DRIVE_MOTOR_REVERSED = false;
        public static final boolean TURNING_MOTOR_REVERSED = true;
        public static final int ABSOLUTE_ENCODER_ID = 2;
        public static final double ENCODER_OFFSET = -0.29052734375;
    }

    public static class BackRight {
        public static final int DRIVE_MOTOR_ID = 31;
        public static final int TURN_MOTOR_ID = 32;
        public static final boolean DRIVE_MOTOR_REVERSED = false;
        public static final boolean TURNING_MOTOR_REVERSED = true;
        public static final int ABSOLUTE_ENCODER_ID = 3;
        public static final double ENCODER_OFFSET = 0.188232421875;
    }

    public static class Hook {
        public static final int LEFT_HOOK = 61;
        public static final int RIGHT_HOOK = 62;
        public static final double UP_SPEED = -0.17;
        public static final double DOWN_SPEED = 0.5;
        public static final double MIN = 0.001;
        public static final double MAX = -4.26;
    }

    public static class Shooter {
        public static final int M_ID_LEFT = 8;
        public static final int M_ID_RIGHT = 9;
        public static final double LAUNCH_SPEED = -0.4;
    }

    public static class PathPlannerConstants {
        // public static final HolonomicPathFollowerConfig
        // HOLONOMIC_PATH_FOLLOWER_CONFIG
        // = new HolonomicPathFollowerConfig(
        // new PIDConstants(5,0,0),
        // new PIDConstants(5,0,0),
        // DriveConstants.PHYSICAL_MAX_SPEED_MPS,
        // 0, //max distance to wheel
        // new ReplanningConfig());
    }

    public static class Elevator {
        public static final int M_ID_LEFT = 1;
        public static final int M_ID_RIGHT = 2;
        public static final double MOTOR_SPEED = -0.2;
        public static final double ANTI_GRAVITY = -0.02;
    }

    public static class Keybindings {
        public static final int BUMPER_LEFT = XboxController.Button.kLeftBumper.value;
        public static final int BUMPER_RIGHT = XboxController.Button.kRightBumper.value;
        public static final int BUTTON_A = XboxController.Button.kA.value;
        public static final int BUTTON_B = XboxController.Button.kB.value;
        public static final int BUTTON_X = XboxController.Button.kX.value;
        public static final int BUTTON_Y = XboxController.Button.kY.value;
        public static final int BUTTON_BACK = XboxController.Button.kBack.value;
        public static final int BUTTON_START = XboxController.Button.kStart.value;
        public static final int BUTTON_LEFT_STICK = XboxController.Button.kLeftStick.value;
        public static final int BUTTON_RIGHT_STICK = XboxController.Button.kRightStick.value;
        public static final int DPAD_UP = 0;
        public static final int DPAD_DOWN = 180;
        public static final int DPAD_LEFT = 270;
        public static final int DPAD_RIGHT = 90;
    }
}
