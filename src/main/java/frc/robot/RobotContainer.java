package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.controlschemes.CharacterizingScheme;
// import frc.controlschemes.CharacterizingScheme;
import frc.controlschemes.SwerveDriveScheme;
import frc.maps.Constants;
import frc.maps.RobotMap;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    SwerveDrive swerveDrive;
    // Leds led;
    // Elevator elevator = new Elevator();
    // Shooter shooter = new Shooter();
    // Intake intake = new Intake();
    // Climber climber = new Climber();
    // Wrist wrist = new Wrist();

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
        // led = new Leds(RobotMap.LED_PORT, RobotMap.LED_LENGTH);

        // initialize controller schemes here
        SwerveDriveScheme.configure(swerveDrive, 0);

        // CharacterizingScheme.configure(swerveDrive, 0);

        // Testing.configure(swerveDrive, 0);

        // fix this with a new subsystem
        // eventMap.put("toggle", Commands.runOnce(() -> swerveDrive.toggleEvent(), null
        // ));

        diagnosticsInit();
        // NamedCommands.registerCommand("Test", elevator.elevatorToSetpoint(0));
        // NamedCommands.registerCommand("Shoot", shooter.shoot());
        // NamedCommands.registerCommand("Rev Shooter", shooter.rev());
        // NamedCommands.registerCommand("Intake", intake.intake(0.5));
        // NamedCommands
        //         .registerCommand("Target Intake",
        //                 parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_INTAKE),
        //                         wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_INTAKE)));
        // NamedCommands
        //         .registerCommand("Target Shoot",
        //                 parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_SHOOT),
        //                         wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT)));

        // NamedCommands
        //         .registerCommand("Target Amp",
        //                 parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP),
        //                                                 wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP)));

                                
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