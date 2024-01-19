package frc.controlschemes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

public class CharacterizingScheme implements ControlScheme{
    public static void configure(SwerveDrive swerveDrive, int port){
        configureButtons(swerveDrive, port);
    }

    /**
     * Configures buttons and their respective commands.
     * @param swerveDrive The SwerveDrive object being configured.
     * @param port The controller port of the driving controller.
     */
    private static void configureButtons(SwerveDrive swerveDrive, int port){
    
             new JoystickButton(controllers[port], ControlMap.A_BUTTON)
            .onTrue(swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        new JoystickButton(controllers[port], ControlMap.B_BUTTON)
            .onTrue(swerveDrive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        new JoystickButton(controllers[port], ControlMap.X_BUTTON)
            .onTrue(swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kForward)); 
        new JoystickButton(controllers[port], ControlMap.Y_BUTTON)
            .onTrue(swerveDrive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }


}
