package frc.diagnostics;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandRunner extends ShuffleManager {
    public CommandRunner(String tab, String title, Command command) {
        Shuffleboard.getTab(tab).add(title, command).withWidget(BuiltInWidgets.kCommand).withPosition(pos.x, pos.y);
    }
}
