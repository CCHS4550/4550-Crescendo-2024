package frc.maps;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    public static class MotorConstants {

        // public static final int FRONT_RIGHT_DRIVE = 6;
        // public static final boolean FRONT_RIGHT_DRIVE_REVERSE = false;
        // public static final double FRONT_RIGHT_DRIVE_ENCODER = 1;
        // public static final int FRONT_RIGHT_TURN = 5;
        // public static final boolean FRONT_RIGHT_TURN_REVERSE = true;
        // public static final double FRONT_RIGHT_TURN_ENCODER = 1;

        // public static final int FRONT_LEFT_DRIVE = 2;
        // public static final boolean FRONT_LEFT_DRIVE_REVERSE = false;
        // public static final double FRONT_LEFT_DRIVE_ENCODER = 1;
        // public static final int FRONT_LEFT_TURN = 1;
        // public static final boolean FRONT_LEFT_TURN_REVERSE = true;
        // public static final double FRONT_LEFT_TURN_ENCODER = 1;

        // public static final int BACK_RIGHT_DRIVE = 7;
        // public static final boolean BACK_RIGHT_DRIVE_REVERSE = false;
        // public static final double BACK_RIGHT_DRIVE_ENCODER = 1;
        // public static final int BACK_RIGHT_TURN = 8;
        // public static final boolean BACK_RIGHT_TURN_REVERSE = true;
        // public static final double BACK_RIGHT_TURN_ENCODER = 1;

        // public static final int BACK_LEFT_DRIVE = 3;
        // public static final boolean BACK_LEFT_DRIVE_REVERSE = false;
        // public static final double BACK_LEFT_DRIVE_ENCODER = 1;
        // public static final int BACK_LEFT_TURN = 4;
        // public static final boolean BACK_LEFT_TURN_REVERSE = true;
        // public static final double BACK_LEFT_TURN_ENCODER = 1;
        public static final int FRONT_RIGHT_DRIVE = 1;
        public static final boolean FRONT_RIGHT_DRIVE_REVERSE = false;
        public static final double FRONT_RIGHT_DRIVE_ENCODER = 1;
        public static final int FRONT_RIGHT_TURN = 2;
        public static final boolean FRONT_RIGHT_TURN_REVERSE = true;
        public static final double FRONT_RIGHT_TURN_ENCODER = 1;

        public static final int FRONT_LEFT_DRIVE = 3;
        public static final boolean FRONT_LEFT_DRIVE_REVERSE = false;
        public static final double FRONT_LEFT_DRIVE_ENCODER = 1;
        public static final int FRONT_LEFT_TURN = 4;
        public static final boolean FRONT_LEFT_TURN_REVERSE = true;
        public static final double FRONT_LEFT_TURN_ENCODER = 1;

        public static final int BACK_RIGHT_DRIVE = 7;
        public static final boolean BACK_RIGHT_DRIVE_REVERSE = false;
        public static final double BACK_RIGHT_DRIVE_ENCODER = 1;
        public static final int BACK_RIGHT_TURN = 8;
        public static final boolean BACK_RIGHT_TURN_REVERSE = true;
        public static final double BACK_RIGHT_TURN_ENCODER = 1;

        public static final int BACK_LEFT_DRIVE = 5;
        public static final boolean BACK_LEFT_DRIVE_REVERSE = false;
        public static final double BACK_LEFT_DRIVE_ENCODER = 1;
        public static final int BACK_LEFT_TURN = 6;
        public static final boolean BACK_LEFT_TURN_REVERSE = true;
        public static final double BACK_LEFT_TURN_ENCODER = 1;

        /**
         * TODO
         * Get all these values from electrical
         */
        public static final int SHOOTER_TOP = 16;
        public static final boolean SHOOTER_TOP_REVERSED = true;

        public static final int SHOOTER_BOTTOM = 15;
        public static final boolean SHOOTER_BOTTOM_REVERSED = false;

        public static final int INDEXER = 13;
        public static final boolean INDEXER_REVERSED = false;

        public static final int INTAKE_BACK = 12;
        public static final boolean INTAKE_RIGHT_REVERSED = false;

        public static final int INTAKE_FRONT = 9;
        public static final boolean INTAKE_LEFT_REVERSED = false;

        public static final int ELEVATOR_RIGHT = 11;
        public static final boolean ELEVATOR_RIGHT_REVERSED = true;

        public static final int ELEVATOR_LEFT = 10;
        public static final boolean ELEVATOR_LEFT_REVERSED = false;

        public static final int WRIST = 14;
        public static final boolean WRIST_REVERSED = true;

        public static final int CLIMBER_RIGHT = 98;
        public static final boolean CLIMBER_RIGHT_REVERSED = false;

        public static final int CLIMBER_LEFT = 99;
        public static final boolean CLIMBER_LEFT_REVERSED = false;

    }

    public static class SwerveConstants {

        // Absolute Encoder Ports
        public static final int FRONT_RIGHT_ABSOLUTE_ENCODER = 0;
        public static final int FRONT_LEFT_ABSOLUTE_ENCODER = 1;
        public static final int BACK_RIGHT_ABSOLUTE_ENCODER = 2;
        public static final int BACK_LEFT_ABSOLUTE_ENCODER = 3;

        public static final double FRONT_RIGHT_ABSOLUTE_ENCODER_OFFSET = 5.55;
        public static final double FRONT_LEFT_ABSOLUTE_ENCODER_OFFSET = 0.613;
        public static final double BACK_RIGHT_ABSOLUTE_ENCODER_OFFSET = 5.77;
        public static final double BACK_LEFT_ABSOLUTE_ENCODER_OFFSET = 0.184;

        // Robot Constants (change with SysId)
        // max speed in free sprint: used in getting velocities of swerve modules
        public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL = 4.72;


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

        //Front to back
        public static final double WHEEL_BASE = Units.inchesToMeters(19.25); // from drive shaft to drive shaft. Previous
                                                                          // was
        //Right to Left                                                            // 27
        public static final double TRACK_WITDTH = Units.inchesToMeters(22.25);

         /** FR FL BR BL. Same as order of swerve module states */
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, -TRACK_WITDTH / 2),
                new Translation2d(WHEEL_BASE / 2, TRACK_WITDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WITDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WITDTH / 2));
    }

    /* To Do */
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 50;

    public class FeedForwardConstants {

        // TODO Do sysid to get values
        public static final double DRIVE_KS = 0.19268;
        public static final double DRIVE_KV = 2.642;
        public static final double DRIVE_KA = 0.55965;

        public static final double TURNKS = 0;
        public static final double TURNKV = 0;

        // * TODO SysId these values */
        public static final double ELEVATOR_KS = 0.17092;
        public static final double ELEVATOR_KG = 0.075178;
        public static final double ELEVATOR_KV = 0.0019272;
        public static final double ELEVATOR_KA = 0.00029478;

        public static final double WRIST_KS = 0.17091;
        public static final double WRIST_KG = 0.32638;
        public static final double WRIST_KV = 0.0017181;
        public static final double WRIST_KA = 0.00024211;
    }

    public class FieldPositionConstants {
        /** Blue Top to Bottom then Red Top to Bottom */
        public static Pose2d[] blueNotePoses = new Pose2d[] { new Pose2d(2.89, 7.0, null), new Pose2d(2.89, 5.53, null),
                new Pose2d(2.89, 4.10, new Rotation2d(0)) };

        // public static Pose2d[] redNotePoses = new Pose2d[]{new Pose2d(2.89, 7.0,
        // null), new Pose2d(2.89, 5.53, null),
        // new Pose2d(2.89, 4.10, new Rotation2d(0))}
    }

    public class MechanismPositions {
        public static double ELEVATOR_INTAKE = 0;
        public static double WRIST_INTAKE = 0;

        public static double ELEVATOR_SHOOT = 0;
        public static double WRIST_SHOOT = 15.619040489196777;
        // public static double WRIST_SHOOT = 8.714315414428711;

        public static double ELEVATOR_AMP = 76.5;
        public static double WRIST_AMP = 51.02395248413086;

        public static double ELEVATOR_HUMAN_PLAYER = 0;
        public static double WRIST_HUMAN_PLAYER = 0;

        public static double ELEVATOR_TOP = 75;

        public static double WRIST_TRAVEL = 20;
    }

    public class RedFieldPositionConstants {
        public static Pose2d SPEAKER_FRONT = new Pose2d(new Translation2d(5.5, 15.2), new Rotation2d(0));
        public static Pose2d SPEAKER_LEFT = new Pose2d(new Translation2d(15.85, 4.4), new Rotation2d(60));
        public static Pose2d SPEAKER_RIGHT = new Pose2d(new Translation2d(15.85, 6.7), new Rotation2d(300));
        public static Pose2d SPEAKER_MIDDLE = new Pose2d(new Translation2d(16.25, 5.5), new Rotation2d(0));
        public static Pose2d[] SPEAKER_POSES = { SPEAKER_FRONT, SPEAKER_LEFT, SPEAKER_RIGHT };
        public static Pose2d AMP = new Pose2d(new Translation2d(14.65, 7.65), new Rotation2d(90));
        public static Pose2d SOURCE = new Pose2d(new Translation2d(1.13, .96), new Rotation2d(240));

        public static Pose2d STAGE_TOP = new Pose2d(new Translation2d(12.4, 4.9),
                new Rotation2d(Units.degreesToRadians(60)));
        public static Pose2d STAGE_BOTTOM = new Pose2d(new Translation2d(12.4, 3.4),
                new Rotation2d(Units.degreesToRadians(300)));
        public static Pose2d STAGE_SIDE = new Pose2d(new Translation2d(10.7, 4.1),
                new Rotation2d(Units.degreesToRadians(180)));

    }

    public class BlueFieldPositionConstants {
        public static Pose2d SPEAKER_FRONT = new Pose2d(new Translation2d(1.35, 5.55), new Rotation2d(180));
        public static Pose2d SPEAKER_LEFT = new Pose2d(new Translation2d(0.7, 6.7), new Rotation2d(240));
        public static Pose2d SPEAKER_RIGHT = new Pose2d(new Translation2d(.75, 4.40), new Rotation2d(120));
        public static Pose2d SPEAKER_MIDDLE = new Pose2d(new Translation2d(0.25, 5.5), new Rotation2d(0));
        public static Pose2d[] SPEAKER_POSES = { SPEAKER_FRONT, SPEAKER_LEFT, SPEAKER_RIGHT };
        public static Pose2d AMP = new Pose2d(new Translation2d(1.82, 7.66), new Rotation2d(90));
        public static Pose2d SOURCE = new Pose2d(new Translation2d(15.5, 1), new Rotation2d(300));
        // public static Rotation2d STAGE_LEFT = new Rotation2d(120);
        // public static Rotation2d STAGE_RIGHT = new Rotation2d(240);
        // public static Rotation2d STAGE_FRONT = new Rotation2d(0);
        public static Pose2d STAGE_TOP = new Pose2d(new Translation2d(4.4, 4.9),
                new Rotation2d(Units.degreesToRadians(120)));// idk if this is important->
                                                             // //AprilTags.aprilTagFieldLayout.getTagPose(AprilTags.BLUE_STAGE_TOP).get().toPose2d().transformBy(new
                                                             // Transform2d(2.0, 0.0,new Rotation2d(0.0)));
        public static Pose2d STAGE_BOTTOM = new Pose2d(new Translation2d(4.4, 3.2),
                new Rotation2d(Units.degreesToRadians(240)));
        public static Pose2d STAGE_SIDE = new Pose2d(new Translation2d(5.9, 4.1), new Rotation2d(0));
    }

    public class XboxConstants {
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

        // D Pad Buttons
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

    public static class Vision {
        public static final String CAMERA_NAME = "FrontCamera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.

        // The layout of the AprilTags on the field

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class cameraOne {
        public static final String CAMERA_ONE_NAME = "FrontCamera";
        public static final Transform3d ROBOT_TO_CAM = new Transform3d(new Translation3d(Inches.of(9.25), Inches.of(-9.75), Inches.of(14.5)),
                new Rotation3d(0, Units.degreesToRadians(35.0), Units.degreesToRadians(180)));
        public static frc.helpers.Vision FRONT_CAMERA = new frc.helpers.Vision(CAMERA_ONE_NAME, ROBOT_TO_CAM);
    }

    /**
     * Gotten from here
     * https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
     */
    public static class AprilTags {
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo
                .loadAprilTagLayoutField();

        public static int BLUE_SOURCE_LEFT = 1;
        public static int BLUE_SOURCE_RIGHT = 2;
        public static int RED_SPEAKER_BOTTOM = 3;
        public static int RED_SPEAKER_TOP = 4;
        public static int RED_AMP = 5;
        public static int BLUE_AMP = 6;
        public static int BLUE_SPEAKER_TOP = 7;
        public static int BLUE_SPEAKER_BUTTON = 8;
        public static int RED_SOURCE_LEFT = 9;
        public static int RED_SOURCE_RIGHT = 10;
        public static int RED_STAGE_BOTTOM = 11;
        public static int RED_STAGE_TOP = 12;
        public static int RED_STAGE_SIDE = 13;
        public static int BLUE_STAGE_SIDE = 14;
        public static int BLUE_STAGE_TOP = 15;
        public static int BLUE_STAGE_BOTTOM = 16;
    }

    public static Pose2d mirrorPose(Pose2d bluePose) {
        return new Pose2d(
                Constants.AprilTags.aprilTagFieldLayout.getFieldLength() - bluePose.getX(),
                bluePose.getY(),
                Rotation2d.fromRadians(Math.PI - bluePose.getRotation().getRadians()));
    }

    public static class ElevatorConstants {
        public static double ELEVATOR_HEIGHT = 19.345;
        public static double ELEVATOR_ANGLE = 0.611;
    }
}
