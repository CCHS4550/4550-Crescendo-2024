package frc.robot.autonomous;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.maps.RobotMap;
import frc.robot.subsystems.Elevator;

public class ElevatorToSetPoint extends Command {
    private Elevator elevator;
    private double setpoint;
    private TrapezoidProfile currentProfile;
    private ElevatorFeedforward feedforward;
    private PIDController elevatorPIDController;

    /** Creates a new ElevatorToSetpointWithFeedForward. */
    public ElevatorToSetPoint(
            Elevator elevatorSubsystem, double setPoint) {
        this.elevator = elevatorSubsystem;
        this.setpoint = setPoint;
        this.feedforward = new ElevatorFeedforward(
                RobotMap.ELEVATOR_KS,
                RobotMap.ELEVATOR_KG,
                RobotMap.ELEVATOR_KV,
                RobotMap.ELEVATOR_KA);
        this.elevatorPIDController = new PIDController(0.3, 0, 0);
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.setGoal(setpoint);
        elevator.setSetpoint(
                new TrapezoidProfile.State(elevator.getElevatorPosition(), 0));

        SmartDashboard.putNumber("Elevator Goal Position", elevator.getGoal().position);
        SmartDashboard.putNumber(
                "Elevator Setpoint Position", elevator.getSetpoint().position);
    }

    // Called every time the schseduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentProfile = new TrapezoidProfile(
                elevator.getConstraints());
        double currentPosition = elevator.getElevatorPosition();

        TrapezoidProfile.State nextSetpoint = currentProfile.calculate(0.02, elevator.getSetpoint(), elevator.getGoal());

        double feedForwardPower = feedforward.calculate(nextSetpoint.velocity);

        elevator.setSetpoint(nextSetpoint);

        SmartDashboard.putNumber("Trapezoid Position", nextSetpoint.position);

        elevatorPIDController.setSetpoint(nextSetpoint.position);

        double elevatorPower = elevatorPIDController.calculate(currentPosition);
        // elevator.setElevatorVoltage(Units.Volts.of(elevatorPower + feedForwardPower));

        elevator.setPosition(nextSetpoint.position, feedForwardPower);
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