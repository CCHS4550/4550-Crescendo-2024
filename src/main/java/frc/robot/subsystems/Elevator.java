package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import frc.maps.RobotMap;

public class Elevator extends SubsystemBase {

    SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(5), Seconds.of(5)),
            new SysIdRoutine.Mechanism(
                    (voltage) -> setElevatorVoltage(voltage),
                    null, // No log consumer, since data is recorded by URCL
                    this));

    private CCSparkMax elevatorMotorOne = new CCSparkMax("Elevator One", "EO", Constants.MotorConstants.ELEVATOR_ONE,
            MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_ONE_REVERSED);
    private CCSparkMax elevatorMotorTwo = new CCSparkMax("Elevator Two", "ET", Constants.MotorConstants.ELEVATOR_TWO,
            MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_TWO_REVERSED);

    private ElevatorFeedforward elevatorMotorFeedforward;

    private PIDController elevatorPidController;

    private TrapezoidProfile.Constraints constraints;

    private TrapezoidProfile profile;

    private TrapezoidProfile.State setPoint, goal;

    DigitalInput limitSwitch = new DigitalInput(0);

    public Elevator() {
        elevatorMotorFeedforward = new ElevatorFeedforward(
                Constants.FeedForwardConstants.ELEVATOR_KS,
                Constants.FeedForwardConstants.ELEVATOR_KG,
                Constants.FeedForwardConstants.ELEVATOR_KV,
                Constants.FeedForwardConstants.ELEVATOR_KA);
        elevatorPidController = new PIDController(0.3, 0, 0);

        constraints = new Constraints(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.5));
        profile = new TrapezoidProfile(constraints);
        setPoint = new TrapezoidProfile.State();
        goal = new TrapezoidProfile.State();
    }

    public void targetPosition(double position) {
        setGoal(position);

        TrapezoidProfile.State nextSetpoint = profile.calculate(0.02, getSetpoint(), getGoal());

        double feedForwardPower = elevatorMotorFeedforward.calculate(nextSetpoint.velocity);

        setSetpoint(nextSetpoint);

        SmartDashboard.putNumber("Trapezoid Position", nextSetpoint.position);

        // double elevatorPower = elevatorPidController.calculate(currentPosition);

        elevatorMotorOne.getPIDController().setReference(position, ControlType.kPosition, 0, feedForwardPower,
                ArbFFUnits.kVoltage);

        elevatorMotorTwo.getPIDController().setReference(position, ControlType.kPosition, 0, feedForwardPower,
                ArbFFUnits.kVoltage);
    }

    public void setElevatorVoltage(Measure<Voltage> volts) {
        elevatorMotorOne.setVoltage(volts.magnitude());
        elevatorMotorTwo.setVoltage(volts.magnitude());
    }

    public void setSetpoint(TrapezoidProfile.State setPoint) {
        this.setPoint = setPoint;
    }

    public TrapezoidProfile.State getSetpoint() {
        return setPoint;
    }

    public void setGoal(double goalState) {
        goal = new TrapezoidProfile.State(goalState, 0);
    }

    public State getGoal() {
        return goal;
    }

    public double getElevatorPosition() {
        return edu.wpi.first.math.util.Units.rotationsToRadians(elevatorMotorOne.getPosition());
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    public void resetEncoders() {
        elevatorMotorOne.reset();
        elevatorMotorTwo.reset();
    }

    @Override
    public void periodic() {

    }

    //Sets the elevator to target a setpoint
    public Command elevatorToSetpoint(double setpoint) {
        return this.run(
                () -> this.targetPosition(setpoint)).until(
                        () -> (getSetpoint().position == getGoal().position))
                .onlyIf(() -> (limitSwitch.get() == false));
    }

   // homes the elevator
   public Command home() {
    return sequence(
            this.run(() -> setElevatorVoltage(Volts.of(elevatorMotorFeedforward.calculate(-0.5))))
              .until(() -> (limitSwitch.get())),
            elevatorToSetpoint(1),
            waitSeconds(1),
            elevatorToSetpoint(0)
    );
}

    /**
     * Used only in characterizing. Don't touch this.
     * 
     * @param direction
     * @return the quasistatic characterization test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Used only in characterizing. Don't touch this.
     * 
     * @param direction
     * @return the dynamic characterization test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

}
