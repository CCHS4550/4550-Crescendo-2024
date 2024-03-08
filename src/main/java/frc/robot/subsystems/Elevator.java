package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import frc.maps.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {

    SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(5), Seconds.of(4),
                    (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                    (voltage) -> setElevatorVoltage(voltage),
                    null, // No log consumer, since data is recorded by URCL
                    this));

    private CCSparkMax elevatorMotorRight = new CCSparkMax("Elevator One", "EO",
            Constants.MotorConstants.ELEVATOR_RIGHT,
            MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_RIGHT_REVERSED);
    private CCSparkMax elevatorMotorLeft = new CCSparkMax("Elevator Two", "ET", Constants.MotorConstants.ELEVATOR_LEFT,
            MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.ELEVATOR_LEFT_REVERSED);

    private GenericEntry elevatorRightPosEntry, elevatorLeftPosEntry;

    private ShuffleboardLayout elevatorPosEntries = Shuffleboard.getTab("Encoders")
            .getLayout("Elevator Positions", BuiltInLayouts.kGrid).withSize(1, 2);

    private ElevatorFeedforward elevatorMotorFeedforward;

    private TrapezoidProfile.Constraints constraints;

    private TrapezoidProfile profile;

    private TrapezoidProfile.State setPoint, goal;

    DigitalInput limitSwitchBottom = new DigitalInput(1);
    DigitalInput limitSwitchTop = new DigitalInput(2);

    PIDController elevatorPid = new PIDController(.8, 0, 0);

    public Elevator() {
        resetEncoders();
        elevatorRightPosEntry = Shuffleboard.getTab("Encoders")
                .getLayout(elevatorPosEntries.getTitle())
                .add(elevatorMotorRight.getName(), elevatorMotorRight.getPosition()).getEntry();

        elevatorLeftPosEntry = Shuffleboard.getTab("Encoders")
                .getLayout(elevatorPosEntries.getTitle())
                .add(elevatorMotorLeft.getName(), elevatorMotorLeft.getPosition()).getEntry();

        elevatorMotorFeedforward = new ElevatorFeedforward(
                Constants.FeedForwardConstants.ELEVATOR_KS,
                Constants.FeedForwardConstants.ELEVATOR_KG,
                Constants.FeedForwardConstants.ELEVATOR_KV,
                Constants.FeedForwardConstants.ELEVATOR_KA);

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

        SmartDashboard.putNumber("Setpoint", getSetpoint().position);
        double pidCalc = elevatorPid.calculate(elevatorMotorRight.getPosition(), position);

        // double elevatorPower = elevatorPidController.calculate(currentPosition);

        // elevatorMotorRight.getPIDController().setReference(position,
        // ControlType.kPosition, 0, feedForwardPower,
        // ArbFFUnits.kVoltage);
        // elevatorMotorLeft.getPIDController().setReference(position,
        // ControlType.kPosition, 0, feedForwardPower,
        // ArbFFUnits.kVoltage);

        elevatorMotorRight.setVoltage(feedForwardPower + pidCalc);
        elevatorMotorLeft.setVoltage(feedForwardPower + pidCalc);

    }

    // Sets the elevator to target a setpoint
    public Command elevatorToSetpoint(double setpoint) {
        return this.runEnd(
                () -> this.targetPosition(setpoint), () -> setElevatorSpeed(0)).until(
                        () -> ((Math.abs(setpoint - elevatorMotorRight.getPosition())) < 0.3));
    }

    // homes the elevator
    public Command home() {
        return sequence(setElevatorDutyCycle(() -> -0.7));
    }

    public void setElevatorVoltage(Measure<Voltage> volts) {
        elevatorMotorRight.setVoltage(volts.magnitude());
        elevatorMotorLeft.setVoltage(volts.magnitude());
    }

    public void setElevatorSpeed(double speed) {
        elevatorMotorRight.set(speed);
        elevatorMotorLeft.set(speed);
    }

    public boolean getLimitSwitchBottom() {
        return limitSwitchBottom.get();
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
        return edu.wpi.first.math.util.Units.rotationsToRadians(elevatorMotorRight.getPosition());
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limit Switch Bottom", limitSwitchBottom.get());
        SmartDashboard.putBoolean("Limit Switch Top", limitSwitchTop.get());
        updateShuffleBoardEncoders();
        if (limitSwitchBottom.get()) {
            resetEncoders();
        }
    }

    /**
     * Used to update the shuffleboard entries for the elevator motors
     */
    public void updateShuffleBoardEncoders() {
        elevatorRightPosEntry.setDouble(elevatorMotorRight.getPosition());
        elevatorLeftPosEntry.setDouble(elevatorMotorLeft.getPosition());
    }

    public double elevatorElevation() {
        return Math.sin(ElevatorConstants.ELEVATOR_ANGLE) * elevatorMotorRight.getPosition();
    }

    public void resetEncoders() {
        elevatorMotorRight.reset();
        elevatorMotorLeft.reset();
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

    public Command halt() {
        return Commands.runOnce(() -> {
        }, this);
    }

    // Duty Cycle Code
    public Command dutyHome() {
        return sequence(
                // setElevatorDutyCycle(() -> -0.2),
                // elevatorToSetpoint(20),
                // waitSeconds(1),
                elevatorToSetpoint(0));
    }

    public Command setElevatorDutyCycle(DoubleSupplier speed) {
        return this.runEnd(() -> setElevatorSpeed(speed.getAsDouble()), () -> setElevatorSpeed(0))
                .until(() -> (limitSwitchBottom.get() && speed.getAsDouble() < 0
                        || limitSwitchTop.get() && speed.getAsDouble() > 0));
    }

    // public void setElevatorVoltageSpeed(double input) {
    // double ffValue = elevatorMotorFeedforward.calculate(input);
    // SmartDashboard.putNumber("FF", ffValue);
    // elevatorMotorRight.setVoltage(ffValue);
    // elevatorMotorLeft.setVoltage(ffValue);
    // }

    // public Command runElevatorVoltage(Measure<Voltage> volts) {
    // return this.runEnd(() -> setElevatorVoltage(volts), () ->
    // setElevatorVoltage(Volts.of(0)))
    // .until(() -> (limitSwitchBottom.get() && volts.magnitude() < 0));
    // }

    // public Command runElevatorVoltageSpeed(DoubleSupplier input) {
    // // this.input = input;
    // return this.runEnd(() -> setElevatorVoltageSpeed(input.getAsDouble()), () ->
    // setElevatorVoltageSpeed(0))
    // .until(() -> (limitSwitchBottom.get() && input.getAsDouble() < 0));
    // }

}
