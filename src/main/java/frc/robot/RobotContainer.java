package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.controlschemes.CharacterizingScheme;
import frc.controlschemes.SwerveDriveScheme;
import frc.maps.Constants;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    SwerveDrive swerveDrive;

    /** Event map for path planner */
    public static HashMap<String, Command> eventMap = new HashMap<>();
    /** Command List for auto paths in SmartDashBoard */

    LoggedDashboardChooser<Command> autoCommands = new LoggedDashboardChooser<Command>("Auto Commands");
    // private final String[] paths = {"outback","out","outturn","turn","meters",
    // "EventTest"};
    // private static String[] paths = { "move" };
    private final SendableChooser<Command> autoChooser;
    // private final LoggedDashboardChooser<Command> autoChooser

    Field2d ff;

    public RobotContainer() {
        // initialize subsytems here
        swerveDrive = new SwerveDrive();

        // initialize controller schemes here
        SwerveDriveScheme.configure(swerveDrive, 0);


        diagnosticsInit();
        // NamedCommands.registerCommand("Test", elevator.elevatorToSetpoint(0));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    public void diagnosticsInit() {
        // for (String pathName : paths) {
        // autoCommands.addOption(pathName,
        // followPathPlanner(pathName).withName(pathName));
        // }
        // this is the default command to prevent accidents of running the wrong auto
        // autoCommands.addDefaultOption("Nothing",
        // Commands.run(() -> swerveDrive.printWorld(),
        // swerveDrive).withName("Nothing"));

        // SmartDashboard.putData("Auto", autoCommands.getSendableChooser());
        // autoCommands.addOption("Move Straight",
        // swerveDrive.moveCommand().withName("Move Straight"));

        Shuffleboard.getTab("Diagnostics").add("SwerveDrive", swerveDrive);
        // Shuffleboard.getTab("Config").add("Run Auto",
        // getAutoCommand()).withWidget(BuiltInWidgets.kCommand);
    }

    public Command getAutoCommand() {
        // return autoCommands.get();
        return autoChooser.getSelected();
    }
}