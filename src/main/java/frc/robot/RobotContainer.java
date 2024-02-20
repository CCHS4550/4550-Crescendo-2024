package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.subsystems.Blinkin;

public class RobotContainer {
        private static CommandXboxController controller;
        private Blinkin blinkin;
        public RobotContainer() {
                blinkin = new Blinkin();
                controller = new CommandXboxController(0);
                controller.a().onTrue(Commands.runOnce(() -> {
                        blinkin.toggleColor();
                        blinkin.printColor();
                }, blinkin));
        }

        public Command getAutoCommand() {
                return Commands.waitSeconds(1);
        }
}