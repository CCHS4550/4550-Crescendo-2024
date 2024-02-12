package frc.controlschemes;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.Constants;
import frc.maps.ControlMap;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

/**
 * Control scheme for swerve drive. Includes movement, the toggle between
 * field centric and robot centric, and a button to zero the gyro.
 */
public class SwerveDriveScheme implements ControlScheme {
    private static boolean fieldCentric = true;
    private static boolean orientationLocked = false;
    private static double orientationLockAngle;
    private static BooleanSupplier fieldCentricSupplier = () -> {
        return fieldCentric;
    };

    /**
     * Configures the basic driving as well as buttons.
     * 
     * @param swerveDrive The SwerveDrive object being configured.
     * @param port        The controller port of the driving controller.
     */
    public static void configure(SwerveDrive swerveDrive, int port) {
        Shuffleboard.getTab("Diagnostics").getLayout("Swerve", "List").add("isCentric", fieldCentric)
                .withWidget(BuiltInWidgets.kBooleanBox);
        Shuffleboard.getTab("Diagnostics").addBoolean("Field Centric", fieldCentricSupplier)
                .withWidget(BuiltInWidgets.kToggleSwitch);

        SlewRateLimiter xRateLimiter = new SlewRateLimiter(Constants.SwerveConstants.DRIVE_RATE_LIMIT, -Constants.SwerveConstants.DRIVE_RATE_LIMIT, 0);
        SlewRateLimiter yRateLimiter = new SlewRateLimiter(Constants.SwerveConstants.DRIVE_RATE_LIMIT, -Constants.SwerveConstants.DRIVE_RATE_LIMIT, 0);
        SlewRateLimiter turnRateLimiter = new SlewRateLimiter(Constants.SwerveConstants.TURN_RATE_LIMIT);

        PIDController orientationLockPID = new PIDController(.5, 0, 0);

        swerveDrive.setDefaultCommand(new RunCommand(() -> {

            // Set x, y, and turn speed based on joystick inputs
            double xSpeed = -OI.axis(port, Constants.XboxConstants.L_JOYSTICK_VERTICAL) * .75
                    * (OI.axis(0, Constants.XboxConstants.RT) > 0.5 ? 0.5 : (OI.axis(0, Constants.XboxConstants.LT) > 0.5 ? (4 / 3) : 1))
                    * Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL;
            double ySpeed = -OI.axis(port, Constants.XboxConstants.L_JOYSTICK_HORIZONTAL) * .75
                    * (OI.axis(0, Constants.XboxConstants.RT) > 0.5 ? 0.5 : (OI.axis(0, Constants.XboxConstants.LT) > 0.5 ? (4 / 3) : 1))
                    * Constants.SwerveConstants.MAX_DRIVE_SPEED_METERS_PER_SECOND_THEORETICAL;
            double turnSpeed = 0;
            if (!orientationLocked) {
                turnSpeed = -OI.axis(port, Constants.XboxConstants.R_JOYSTICK_HORIZONTAL);
            } else {
                turnSpeed = orientationLockPID.calculate(swerveDrive.getRotation2d().getRadians(), orientationLockAngle)
                        * 2;
            }
            turnSpeed *= 2.0 * Math.PI;

            // Limits acceleration and speed
            // Possibly change the speed limiting to somewhere else (maybe a normalize
            // function)
            xSpeed = xRateLimiter.calculate(xSpeed);
            ySpeed = yRateLimiter.calculate(ySpeed);
            turnSpeed = turnRateLimiter.calculate(turnSpeed);

            // Constructs desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            if (fieldCentric) {
                // Relative to field
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed,
                        swerveDrive.getRotation2d());
                // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 0, turnSpeed,
                // swerveDrive.getRotation2d());
            } else {
                // Relative to robot
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
            }
            chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

            SwerveModuleState[] moduleStates;
            // Convert chassis speeds to individual module states
            moduleStates = Constants.SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
           
            swerveDrive.setModuleStates(moduleStates);

        }, swerveDrive).withName("Swerve Controller Command"));
        configureButtons(swerveDrive, port);
    }

    /**
     * Configures buttons and their respective commands.
     * 
     * @param swerveDrive The SwerveDrive object being configured.
     * @param port        The controller port of the driving controller.
     */
    private static void configureButtons(SwerveDrive swerveDrive, int port) {
        new JoystickButton(controllers[port], ControlMap.B_BUTTON)
                .onTrue(new InstantCommand(() -> toggleFieldCentric()));
        new JoystickButton(controllers[port], ControlMap.A_BUTTON)
                .onTrue(new InstantCommand(() -> swerveDrive.zeroHeading()));
        // new JoystickButton(controllers[port], ControlMap.Y_BUTTON)
        //         .onTrue(new InstantCommand(() -> Commands.run(swerveDrive.pathFindToPathThenFollow("Middle to Shoot")), swerveDrive) );
        new JoystickButton(controllers[port], ControlMap.X_BUTTON)
                .onTrue(new InstantCommand(() -> toggleOrientationLock(swerveDrive)))
                .onFalse(new InstantCommand(() -> toggleOrientationLock(swerveDrive)));
    }

    /**
     * Toggle field centric and robot centric driving.
     */
    private static void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    private static void toggleOrientationLock(SwerveDrive swerveDrive) {

        orientationLocked = !orientationLocked;
        if (orientationLocked) {
            orientationLockAngle = swerveDrive.getRotation2d().getRadians();
        }
    }
}