package frc.controlschemes;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.Constants;
import frc.maps.ControlMap;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

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

    public static void configureButtons(int port, Intake intake, Shooter shooter, Elevator elevator, Wrist wrist) {
        buttonBoard.button(1).onTrue(parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_INTAKE),
                        wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_INTAKE)).withName("Target Intake"));
                
        buttonBoard.button(2).onTrue(parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_SHOOT),
                        wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT)).withName("Target Shoot"));
                
        buttonBoard.button(3).onTrue(parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_SHOOT),
                        wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT)).withName("Target Shoot"));        
    }

}
// :)