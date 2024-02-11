package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorToSetPoint extends Command {
    private Elevator elevator;
    private double setpoint;

    /** Creates a new ElevatorToSetpointWithFeedForward. */
    public ElevatorToSetPoint(
            Elevator elevatorSubsystem, double setpoint) {
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the schseduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.targetPosition(setpoint);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (elevator.getSetpoint().position == elevator.getGoal().position);
    }
}