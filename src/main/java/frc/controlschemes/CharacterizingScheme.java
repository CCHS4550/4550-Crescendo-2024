package frc.controlschemes;

import edu.wpi.first.wpilibj.motorcontrol.SD540;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.ControlScheme;
import frc.maps.ControlMap;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

public class CharacterizingScheme implements ControlScheme{
    public static void configure(SwerveDrive sd, Elevator elevator,Wrist wrist, int port){
        configureButtons(sd,elevator,wrist, port);
    }

    /**
     * Configures buttons and their respective commands.
     * @param swerveDrive The SwerveDrive object being configured.
     * @param port The controller port of the driving controller.
     */
    private static void configureButtons(SwerveDrive sd, Elevator elevator, Wrist wrist, int port){
    
             new JoystickButton(controllers[port], ControlMap.A_BUTTON)
            .onTrue(sd.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        new JoystickButton(controllers[port], ControlMap.B_BUTTON)
            .onTrue(sd.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        new JoystickButton(controllers[port], ControlMap.X_BUTTON)
            .onTrue(sd.sysIdDynamic(SysIdRoutine.Direction.kForward)); 
        new JoystickButton(controllers[port], ControlMap.Y_BUTTON)
            .onTrue(sd.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // new JoystickButton(controllers[port], ControlMap.RB_BUTTON).onTrue(Commands.runOnce(() -> swerveDrive.zeroHeading(), swerveDrive));
        // new JoystickButton(controllers[port], ControlMap.A_BUTTON)
        //     .onTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // new JoystickButton(controllers[port], ControlMap.B_BUTTON)
        //     .onTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // new JoystickButton(controllers[port], ControlMap.X_BUTTON)
        //     .onTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward)); 
        // new JoystickButton(controllers[port], ControlMap.Y_BUTTON)
        //     .onTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // new JoystickButton(controllers[port], ControlMap.A_BUTTON)
        //     .onTrue(wrist.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // new JoystickButton(controllers[port], ControlMap.B_BUTTON)
        //     .onTrue(wrist.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // new JoystickButton(controllers[port], ControlMap.X_BUTTON)
        //     .onTrue(wrist.sysIdDynamic(SysIdRoutine.Direction.kForward)); 
        // new JoystickButton(controllers[port], ControlMap.Y_BUTTON)
        //     .onTrue(wrist.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }


}
