package frc.robot;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.controlschemes.MechanismScheme;
import frc.controlschemes.SwerveDriveScheme;
import frc.controlschemes.Testing;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;

public class RobotContainer {
    SwerveDrive swerveDrive = new SwerveDrive();
    /** Event map for path planner */
    public static HashMap<String, Command> eventMap = new HashMap<>();
    /** Command List for auto paths in SmartDashBoard */
    
    LoggedDashboardChooser<Command> autoCommands = new LoggedDashboardChooser<Command>("Auto Commands");
    private final String[] paths = {"outback","out","outturn","turn","meters", "EventTest"};
    // private static String[] paths = { "move" };
    


    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    swerveDrive::getPose, // Pose2d supplier
    swerveDrive::setOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
    RobotMap.DRIVE_KINEMATICS, // SwerveDriveKinematics
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    swerveDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    swerveDrive // The drive subsystem. Used to properly set the requirements of path following commands
);




Field2d ff;
    
    public RobotContainer() {
        SwerveDriveScheme.configure(swerveDrive, 0);
        // Testing.configure(swerveDrive, 0);

        //fix this with a new subsystem
        eventMap.put("toggle", Commands.runOnce(() -> swerveDrive.toggleEvent(), null ));


        diagnosticsInit();
    }

    
    public void diagnosticsInit(){
        for (String pathName : paths) {
            autoCommands.addOption(pathName, followPathPlanner(pathName).withName(pathName));
        }
        //this is the default command to prevent accidents of running the wrong auto
        autoCommands.addDefaultOption("Nothing",
                Commands.run(() -> swerveDrive.printWorld(), swerveDrive).withName("Nothing"));
        
        SmartDashboard.putData("Auto", autoCommands.getSendableChooser());
        autoCommands.addOption("Move Straight", swerveDrive.moveCommand().withName("Move Straight"));
        
        Shuffleboard.getTab("Diagnostics").add("SwerveDrive", swerveDrive);
        // Shuffleboard.getTab("Config").add("Run Auto", getAutoCommand()).withWidget(BuiltInWidgets.kCommand);
    }

    public Command getAutoCommand() {
        return autoCommands.get();
    }

    /**
     * Functionally the same as the Autonomous class method, just less messy.
     */
    public Command followPathPlanner(String pathName) {
        PathPlannerTrajectory traj = PathPlanner.loadPath(pathName,
                RobotMap.AUTO_PATH_CONSTRAINTS);

        return Commands.sequence(
                Commands.waitSeconds(1),
                Commands.runOnce(swerveDrive::resetOdometry, swerveDrive),
                swerveDrive.followTrajectoryCommand(traj, true),
                Commands.runOnce(swerveDrive::stopModules, swerveDrive));
    }

    


/**
 * 
 * @return Command to follow the tutorial path
 */
public Command followTutorialPath() {
    
        // TrajectoryConfig config = new TrajectoryConfig(RobotMap.MAX_SPEED_METERS_PER_SECOND, RobotMap.DRIVE_RATE_LIMIT)
        //         .setKinematics(RobotMap.DRIVE_KINEMATICS);
        // Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), null,
        //         new Pose2d(1, 1, new Rotation2d(0)), config);

        //         PIDController xPID = new PIDController(.5, .15, 0);
        //         PIDController yPID = new PIDController(.5, .15, 0);

                return new InstantCommand();

    }
}


// SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

//     public RobotContainer() {
//         ...
//         m_autoSelector.setDefaultOption("default auto", followPath("Default Path"));
//         m_autoSelector.addOption("other auto", followPath("Other Path"));
//     }
//     public Command getAutonomousCommand() {
//         return m_autoSelector.getSelected();
//     }
//     public Command followPath(String pathName) {
//          return followPath(PathPlanner.loadPath(pathName, constraints));
//     }
//     public Command followPath(PathPlannerTrajectory traj){
//              //as you have it now
//     }


// new CommandSelector("Main Tab", "Auto Selector",
// followPath("DefaultAuto").withName("Default Auto"),
// followPath("OtherAuto").withName("Other Auto")
// );



// SendableChooser<Command> m_autoSelector = new SendableChooser<Command>();

//     public RobotContainer() {
//         ...
//         m_autoSelector.setDefaultOption("default auto", followPath("Default Path"));
//         m_autoSelector.addOption("other auto", followPath("Other Path"));

//         Shuffleboard.getTab("Config")
//             .add("AutoChooser", m_autoSelector)
//             .withWidget("ComboBox Chooser")
//             .withPosition(1, 1)
//             .withSize(1, 1);
//     }
//     public Command getAutonomousCommand() {
//         return m_autoSelector.getSelected();
//     }
//     public Command followPath(String pathName) {
//          return followPath(PathPlanner.loadPath(pathName, constraints));
//     }
//     public Command followPath(PathPlannerTrajectory traj){
//              //as you have it now
//     }
