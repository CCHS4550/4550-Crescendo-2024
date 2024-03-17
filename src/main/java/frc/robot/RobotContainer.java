package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.controlschemes.MechanismScheme;
// import frc.controlschemes.CharacterizingScheme;
import frc.controlschemes.SwerveDriveScheme;
import frc.maps.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    SwerveDrive swerveDrive = new SwerveDrive();
        Elevator elevator = new Elevator();
        Shooter shooter = new Shooter();
        Intake intake = new Intake();
        Wrist wrist = new Wrist();
        Indexer indexer = new Indexer();

    /** Event map for path planner */
    public static HashMap<String, Command> eventMap = new HashMap<>();
    /** Command List for autos in SmartDashBoard */
        private final SendableChooser<Command> autoChooser;
    // private final LoggedDashboardChooser<Command> autoChooser

    Field2d ff;

        Command targetAmp = sequence(parallel(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL),
                        elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP)),
                        wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP)).withName("Target Amp");
        Command ampScore = sequence(
                        parallel(sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL)),
                                        elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP)),
                        wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP),
                        parallel(indexer.indexForTime(0.3, 0.3), shooter.shootForTime(0.3, 1))).withName("Amp Score");

        Command rev = sequence(parallel(shooter.shootForTime(-0.1, 0.1), indexer.indexForTime(-0.1, 0.2)), shooter.shootForTime(1, 1.5)).withName("Rev");
        Command shoot = sequence(parallel(shooter.shootForTime(1, 0.3), indexer.indexForTime(0.3, 0.3))).withName("Shoot without rev");
        Command autoShoot = sequence(parallel(shooter.shootForTime(-0.1, 0.1), indexer.indexForTime(-0.1, 0.2)),
                        shooter.shootForTime(1, 1.5),
                        parallel(shooter.shootForTime(1, 0.3), indexer.indexForTime(0.3, 0.3))).withName("Shoot");

        Command subWooferShoot = sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT), autoShoot)
                        .withName("Subwoofer Shoot");

        Command runIntake = parallel(intake.intake(() -> -1), indexer.index(() -> 0.3), shooter.shoot(() -> -0.1));
    
    public RobotContainer() {
        // initialize subsytems here
        
        // initialize controller schemes here
        SwerveDriveScheme.configure(swerveDrive, shooter, indexer, 0);
MechanismScheme.configure(intake, shooter, elevator, wrist, indexer, 1);

        // CharacterizingScheme.configure(swerveDrive, elevator, wrist, 0);

        diagnosticsInit();
NamedCommands.registerCommand("Home", wrist.home());
                NamedCommands.registerCommand("Shoot", autoShoot);
                NamedCommands.registerCommand("Run Intake", runIntake);
                NamedCommands.registerCommand("Rev", rev);
                NamedCommands.registerCommand("Shoot without rev", shoot);
                // NamedCommands
                // .registerCommand("Target Intake",
                // parallel(elevator.elevatorToSetpoint(
                // Constants.MechanismPositions.ELEVATOR_INTAKE),
                // wrist.wristToSetpoint(
                // Constants.MechanismPositions.WRIST_INTAKE)));
                NamedCommands.registerCommand("Target Intake", sequence(elevator.home(), wrist.home()));
                NamedCommands
                                .registerCommand("Target Shoot",
                                                parallel(elevator.elevatorToSetpoint(
                                                                Constants.MechanismPositions.ELEVATOR_SHOOT),
                                                                wrist.wristToSetpoint(
                                                                                Constants.MechanismPositions.WRIST_SHOOT)));
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
        // return autoCommands.get();
        return autoChooser.getSelected();
    }

    /**
     * Functionally the same as the Autonomous class method, just less messy.
     */
    // public Command followPathPlanner(String pathName) {
    //     PathPlannerTrajectory traj = PathPlanner.loadPath(pathName,
    //             RobotMap.AUTO_PATH_CONSTRAINTS);

    //     return Commands.sequence(
    //             Commands.waitSeconds(1),
    //             Commands.runOnce(swerveDrive::resetOdometry, swerveDrive),
    //             swerveDrive.followTrajectoryCommand(traj, true),
    //             Commands.runOnce(swerveDrive::stopModules, swerveDrive));
    // }
}