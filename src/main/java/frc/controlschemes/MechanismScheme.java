package frc.controlschemes;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.helpers.ControlScheme;
import frc.maps.Constants;
import frc.robot.subsystems.SwerveDrive;


public class MechanismScheme implements ControlScheme {
    private static CommandGenericHID buttonBoard;

    public static void configure(Subsystem subsystem, int port) {
        buttonBoard = new CommandGenericHID(port);
        // arm.setDefaultCommand(Commands.run(() -> arm.moveArm(OI.axis(1,
        // ControlMap.L_JOYSTICK_VERTICAL) * 0.5,
        // OI.axis(1, ControlMap.R_JOYSTICK_VERTICAL) * 0.5), arm));

        // intake.setDefaultCommand(Commands.run(() -> intake.spin(
        // OI.axis(1, ControlMap.LT), OI.axis(1, ControlMap.RT)), intake));

        // new JoystickButton(OI.joystickArray[1], ControlMap.A_BUTTON)
        // .onTrue(Commands.run(() -> intake.toggleReverse(true),
        // intake)).onFalse(Commands.run(() -> intake.toggleReverse(false), intake));
    }

    public static void configureButtons(int port,SwerveDrive swerveDrive) {

    }

}
