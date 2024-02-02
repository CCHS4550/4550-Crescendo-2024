package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.subsystems.Limelight;

public class RobotContainer {
    Limelight limelight = new Limelight("limelight", new Transform3d());
    public RobotContainer() {
        limelight.setDefaultCommand(Commands.run(() -> {System.out.println(limelight.getCameraPoseTargetSpace());}, limelight));
    }
    public Command getAutoCommand() {
        return new InstantCommand();
    }
}