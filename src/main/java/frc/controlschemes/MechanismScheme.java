package frc.controlschemes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MechanismScheme implements ControlScheme  {
    public static void configure(Subsystem subsystem) {
        // arm.setDefaultCommand(Commands.run(() -> arm.moveArm(OI.axis(1, ControlMap.L_JOYSTICK_VERTICAL) * 0.5,
        //         OI.axis(1, ControlMap.R_JOYSTICK_VERTICAL) * 0.5), arm));

        // intake.setDefaultCommand(Commands.run(() -> intake.spin(
        //         OI.axis(1, ControlMap.LT), OI.axis(1, ControlMap.RT)), intake));

        // new JoystickButton(OI.joystickArray[1], ControlMap.A_BUTTON)
        //         .onTrue(Commands.run(() -> intake.toggleReverse(true), intake)).onFalse(Commands.run(() -> intake.toggleReverse(false), intake));
    }

    public static void configureButtons(int port, Intake intake, Shooter shooter, Elevator elevator){
        new JoystickButton(controllers[port], 1)
                .onTrue(elevator.elevatorToSetpoint(0));
    }
}
// :)