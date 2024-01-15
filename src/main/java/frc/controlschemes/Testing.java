package frc.controlschemes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

public class Testing implements ControlScheme {
    public static void configure(SwerveDrive swerveDrive, int port) {
        swerveDrive.setDefaultCommand(new RunCommand(() -> {
            SwerveModuleState[] moduleStates = {
                new SwerveModuleState(1, new Rotation2d(0,0)),
                new SwerveModuleState(1, new Rotation2d(0,0)),
                new SwerveModuleState(1, new Rotation2d(0,0)),
                new SwerveModuleState(1, new Rotation2d(0,0))
            };

            swerveDrive.setModuleStates(moduleStates);
            // swerveDrive.test(OI.axis(port, ControlMap.L_JOYSTICK_VERTICAL),
            //         OI.axis(port, ControlMap.R_JOYSTICK_HORIZONTAL));
            // swerveDrive.printAbsoluteEncoders();
            // SwerveModuleState[] states = {
            // new SwerveModuleState(OI.axis(port, ControlMap.L_JOYSTICK_VERTICAL), new
            // Rotation2d(OI.axis(port, ControlMap.R_JOYSTICK_HORIZONTAL))),
            // new SwerveModuleState(OI.axis(port, ControlMap.L_JOYSTICK_VERTICAL), new
            // Rotation2d(OI.axis(port, ControlMap.R_JOYSTICK_HORIZONTAL))),
            // new SwerveModuleState(OI.axis(port, ControlMap.L_JOYSTICK_VERTICAL), new
            // Rotation2d(OI.axis(port, ControlMap.R_JOYSTICK_HORIZONTAL))),
            // new SwerveModuleState(OI.axis(port, ControlMap.L_JOYSTICK_VERTICAL), new
            // Rotation2d(OI.axis(port, ControlMap.R_JOYSTICK_HORIZONTAL)))
            // };
            // swerveDrive.setModuleStates(states);
        },
                swerveDrive));
        configureButtons(swerveDrive, port);
    }

    public static void configureButtons(SwerveDrive swerveDrive, int port) {
        new JoystickButton(controllers[port], ControlMap.A_BUTTON)
                .onTrue(new InstantCommand(() -> swerveDrive.resetAbsoluteEncoders()));
        new JoystickButton(controllers[port], ControlMap.B_BUTTON)
                .onTrue(new InstantCommand(() -> swerveDrive.resetEncoders()));
    }
}
