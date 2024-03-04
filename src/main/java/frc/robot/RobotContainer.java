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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.controlschemes.CharacterizingScheme;
import frc.controlschemes.MechanismScheme;
// import frc.controlschemes.CharacterizingScheme;
import frc.controlschemes.SwerveDriveScheme;
import frc.maps.Constants;
import frc.maps.RobotMap;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    SwerveDrive swerveDrive = new SwerveDrive();
    Leds led;
    Elevator elevator = new Elevator();
    Shooter shooter = new Shooter();
    Intake intake = new Intake();
    // Climber climber = new Climber();
    Wrist wrist = new Wrist();
    Indexer indexer = new Indexer();

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

    Command targetAmp = sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL),
            elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP),
            wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP)).withName("Target Amp");
    Command ampScore = sequence(
            parallel(sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL)),
                    elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP)),
            wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP),
            parallel(indexer.indexForTime(0.3, 0.3), shooter.shootForTime(0.3, 1))).withName("Amp Score");

    Command autoShoot = sequence(parallel(shooter.shootForTime(-0.1, 0.1), indexer.indexForTime(-0.1, 0.2)),
            shooter.shootForTime(1, 1.5),
            parallel(shooter.shootForTime(1, 0.3), indexer.indexForTime(0.3, 0.3))).withName("Auto Shoot");

    Command subWooferShoot = sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT), autoShoot)
            .withName("Subwoofer Shoot");

    public RobotContainer() {
        // initialize subsytems here
        // led = new Leds(RobotMap.LED_PORT, RobotMap.LED_LENGTH);

        // initialize controller schemes here
        SwerveDriveScheme.configure(swerveDrive, 0);
        MechanismScheme.configure(intake, shooter, elevator, wrist, indexer, 1);

        // CharacterizingScheme.configure(swerveDrive, elevator, wrist, 1);

        diagnosticsInit();
        // NamedCommands.registerCommand("Shoot", shooter.shootForTime(1));
        // NamedCommands.registerCommand("Intake", intake.intake(0.5));
        // NamedCommands
        // .registerCommand("Target Intake",
        // parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_INTAKE),
        // wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_INTAKE)));

        // NamedCommands
        // .registerCommand("Target Shoot",
        // parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_SHOOT),
        // wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT)));
        NamedCommands.registerCommand("Target Amp", targetAmp);
        NamedCommands
                .registerCommand("Amp Score",
                        ampScore);

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    public void diagnosticsInit() {

        Shuffleboard.getTab("Subsystems").add("SwerveDrive", swerveDrive);
        Shuffleboard.getTab("Subsystems").add("Elevator", elevator);
        Shuffleboard.getTab("Subsystems").add("Shooter", shooter);
        Shuffleboard.getTab("Subsystems").add("Wrist", wrist);
        Shuffleboard.getTab("Subsystems").add("Intake", intake);
        Shuffleboard.getTab("Subsystems").add("Indexer", indexer);
    }

    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }
}