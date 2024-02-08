package frc.maps;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final double WHEEL_CIRCUMFRENCE = Units.inchesToMeters(4 * Math.PI);

    public class ConverstionConstants {
        // 150/7 rotations of the turn motor to one rotation of the wheel
        // how much of a rotation the wheel turns for one rotation of the turn motor
        public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 7.0 / 150.0;

        // How many radians the wheel pivots for one full rotation of the turn motor
        public static final double TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS = Units
                .rotationsToRadians(TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS);
        public static final double TURN_MOTOR_RADIANS_PER_SECOND = TURN_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS_RADIANS
                / 60.0;

        // 6.75 rotations of the drive motor to one spin of the wheel
        public static final double DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS = 1.0 / 6.75;
        // horizontal distance travelled by one motor rotation
        public static final double HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION = WHEEL_CIRCUMFRENCE
                * DRIVE_MOTOR_ROTATIONS_TO_WHEEL_ROTATIONS;
        public static final double DRIVE_MOTOR_METERS_PER_SECOND_CONVERSION_FACTOR = HORIZONTAL_DISTANCE_TRAVELLED_PER_MOTOR_REVOLUTION
                / 60.0;

    }

    public class MotorConstants {

        public static final int FRONT_RIGHT_DRIVE = 6;
        public static final boolean FRONT_RIGHT_DRIVE_REVERSE = false;
        public static final double FRONT_RIGHT_DRIVE_ENCODER = 1;
        public static final int FRONT_RIGHT_TURN = 5;
        public static final boolean FRONT_RIGHT_TURN_REVERSE = true;
        public static final double FRONT_RIGHT_TURN_ENCODER = 1;

        public static final int FRONT_LEFT_DRIVE = 2;
        public static final boolean FRONT_LEFT_DRIVE_REVERSE = false;
        public static final double FRONT_LEFT_DRIVE_ENCODER = 1;
        public static final int FRONT_LEFT_TURN = 1;
        public static final boolean FRONT_LEFT_TURN_REVERSE = true;
        public static final double FRONT_LEFT_TURN_ENCODER = 1;

        public static final int BACK_RIGHT_DRIVE = 7;
        public static final boolean BACK_RIGHT_DRIVE_REVERSE = false;
        public static final double BACK_RIGHT_DRIVE_ENCODER = 1;
        public static final int BACK_RIGHT_TURN = 8;
        public static final boolean BACK_RIGHT_TURN_REVERSE = true;
        public static final double BACK_RIGHT_TURN_ENCODER = 1;

        public static final int BACK_LEFT_DRIVE = 3;
        public static final boolean BACK_LEFT_DRIVE_REVERSE = false;
        public static final double BACK_LEFT_DRIVE_ENCODER = 1;
        public static final int BACK_LEFT_TURN = 4;
        public static final boolean BACK_LEFT_TURN_REVERSE = true;
        public static final double BACK_LEFT_TURN_ENCODER = 1;

        /**
         * TODO
         * Get all these values from electrical
         */
        public static final int SHOOTER_TOP = 99;
        public static final boolean SHOOTER_TOP_REVERSED = false;

        public static final int SHOOTER_BOTTOM = 99;
        public static final boolean SHOOTER_BOTTOM_REVERSED = false;

        public static final int INDEXER = 99;
        public static final boolean INDEXER_REVERSED = false;

        public static final int INTAKE_TOP = 99;
        public static final boolean INTAKE_TOP_REVERSED = false;

        public static final int INTAKE_BOTTOM = 99;
        public static final boolean INTAKE_BOTTOM_REVERSED = false;

        public static final int ELEVATOR_ONE = 99;
        public static final boolean ELEVATOR_ONE_REVERSED = false;

        public static final int ELEVATOR_TWO = 99;
        public static final boolean ELEVATOR_TWO_REVERSED = false;

        public static final int WRIST = 99;
        public static final boolean WRIST_REVERSED = false;

    }

    public class SwerveConstants{

    // Absolute Encoder Ports
        public static final int FRONT_RIGHT_ABSOLUTE_ENCODER = 3;
        public static final int FRONT_LEFT_ABSOLUTE_ENCODER = 0;
        public static final int BACK_RIGHT_ABSOLUTE_ENCODER = 2;
        public static final int BACK_LEFT_ABSOLUTE_ENCODER = 1;

        public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 5.968 - Math.PI;
        public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 1.752 - Math.PI;
        public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 5.623 - Math.PI;
        public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 2.378 - Math.PI;


    // Robot Constants (change with SysId)
    // max speed in free sprint: used in getting velocities of swerve modules
    public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL = 4.72;

    // new

    // Velocity Limits
    public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND = 5;
    public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 4 * Math.PI;

    // Rate Limiters (acceleration)
    public static final double DRIVE_RATE_LIMIT = MAX_DRIVE_SPEED_METERS_PER_SECOND * 1.5;
    public static final double TURN_RATE_LIMIT = MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    public static final PathConstraints AUTO_PATH_CONSTRAINTS = new PathConstraints(
            MAX_DRIVE_SPEED_METERS_PER_SECOND - 2, DRIVE_RATE_LIMIT - 0.3,
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
            TURN_RATE_LIMIT);
    // public static final PathConstraints AUTO_PATH_CONSTRAINTS = new
    // PathConstraints(4, 3);
    public static final TrapezoidProfile.Constraints thetaControlConstraints = new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, TURN_RATE_LIMIT);

    // Robot Dimensions (relative to wheel locations)
    // Since this robot is a square, no need for 2 values. In a non-square chassis,
    // 2 values needed.
    public static final double WHEEL_BASE = Units.inchesToMeters(24); // from drive shaft to drive shaft. Previous
                                                                      // was
                                                                      // 27

    /** FR FL BR BL. Same as order of swerve module states */
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, -WHEEL_BASE / 2),
            new Translation2d(WHEEL_BASE / 2, WHEEL_BASE / 2),
            new Translation2d(-WHEEL_BASE / 2, -WHEEL_BASE / 2),
            new Translation2d(-WHEEL_BASE / 2, WHEEL_BASE / 2));
    }


    /* To Do */
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 50;

    public class FeedForwardConstants {

        // TODO Do sysid to get values
        public static final double DRIVE_KS = 0.14222;
        public static final double DRIVE_KV = 2.5769;
        public static final double DRIVE_KA = 0.29973;

        public static final double TURNKS = 0;
        public static final double TURNKV = 0;

        // * TODO SysId these values */
        public static final double ELEVATOR_KS = 0;
        public static final double ELEVATOR_KG = 0;
        public static final double ELEVATOR_KV = 0;
        public static final double ELEVATOR_KA = 0;

        public static final double WRIST_KS = 0;
        public static final double WRIST_KG = 0;
        public static final double WRIST_KV = 0;
        public static final double WRIST_KA = 0;
    }

    public class FieldPositionConstants {
        /** Blue Top to Bottom then Red Top to Bottom */
        public static Pose2d[] blueNotePoses = new Pose2d[] { new Pose2d(2.89, 7.0, null), new Pose2d(2.89, 5.53, null),
                new Pose2d(2.89, 4.10, new Rotation2d(0)) };

        // public static Pose2d[] redNotePoses = new Pose2d[]{new Pose2d(2.89, 7.0,
        // null), new Pose2d(2.89, 5.53, null),
        // new Pose2d(2.89, 4.10, new Rotation2d(0))}
    }

    public class RobotPositionConstants {

    }

    public class XboxConstants{
         // Joystick Axises
    public static final int L_JOYSTICK_HORIZONTAL = 0;
    public static final int L_JOYSTICK_VERTICAL = 1;
    public static final int LT = 2;
    public static final int RT = 3;
    public static final int R_JOYSTICK_HORIZONTAL = 4;
    public static final int R_JOYSTICK_VERTICAL = 5;

    // Controller Buttons
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LB_BUTTON = 5;
    public static final int RB_BUTTON = 6;
    public static final int SELECT_BUTTON = 7;
    public static final int START_BUTTON = 8;

    // These buttons are when you push down the left and right circle pad
    public static final int L_JOYSTICK_BUTTON = 9;
    public static final int R_JOYSTICK_BUTTON = 10;

    //D Pad Buttons
    public static final int DPAD_UP = 0;
    public static final int DPAD_UP_RIGHT = 45;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_DOWN_RIGHT = 135;
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_DOWN_LEFT = 225;
    public static final int DPAD_LEFT = 270;
    public static final int DPAD_UP_LEFT = 315;

    // Controller Zeroes
    public static final double ZERO = 0.15;
    }
}
