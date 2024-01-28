package frc.robot.autonomous;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.LimelightHelpers;
import frc.maps.RobotMap;
import frc.robot.subsystems.SwerveDrive;
public class AlignToTag extends Command {
  private final SwerveDrive swerveDrive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToTag(SwerveDrive swerveDrive) {
   this.swerveDrive = swerveDrive;
    
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double turnSpeed = swerveDrive.turnPID.calculate(LimelightHelpers.getTY("Limelight3"), 0);
    // swerveDrive.setModuleStates(RobotMap.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, turnSpeed)));
    swerveDrive.setModuleStates(RobotMap.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, swerveDrive.turnPID.calculate(LimelightHelpers.getTX("Limelight3"), 0))));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
