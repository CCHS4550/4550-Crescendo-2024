package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import frc.maps.RobotMap;

public class Wrist extends SubsystemBase {

    SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(5), Seconds.of(5)),
            new SysIdRoutine.Mechanism(
                    (voltage) -> setWristVoltage(voltage),
                    null, // No log consumer, since data is recorded by URCL
                    this)
            );

    private CCSparkMax wristMotor = new CCSparkMax("Wrist Motor", "WM", Constants.MotorConstants.WRIST,
            MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.WRIST_REVERSED);

    private ElevatorFeedforward wristMotorFeedforward;

    private PIDController wristPidController;

    private TrapezoidProfile.Constraints constraints;

    private TrapezoidProfile profile;

    private TrapezoidProfile.State setPoint, goal;

    DigitalInput limitSwitch = new DigitalInput(0);

    public Wrist() {
        wristMotorFeedforward = new ElevatorFeedforward(
                RobotMap.WRIST_KS,
                RobotMap.WRIST_KG,
                RobotMap.WRIST_KV,
                RobotMap.WRIST_KA);
        wristPidController = new PIDController(0.3, 0, 0);

        constraints = new Constraints(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.5));
        profile = new TrapezoidProfile(constraints);
        setPoint = new TrapezoidProfile.State();
        goal = new TrapezoidProfile.State();
    }

    public void targetPosition(double position) {
        setGoal(position);

        TrapezoidProfile.State nextSetpoint = profile.calculate(0.02, getSetpoint(), getGoal());

        double feedForwardPower = wristMotorFeedforward.calculate(nextSetpoint.velocity);

        setSetpoint(nextSetpoint);

        Logger.recordOutput("Wrist/Trapezoid Position", nextSetpoint.position);

        // double elevatorPower = elevatorPidController.calculate(currentPosition);

        wristMotor.getPIDController().setReference(position, ControlType.kPosition, 0, feedForwardPower,
                ArbFFUnits.kVoltage);

    }

    public void setWristVoltage(Measure<Voltage> volts) {
        wristMotor.setVoltage(volts.magnitude());
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
        return edu.wpi.first.math.util.Units.rotationsToRadians(wristMotor.getPosition());
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return constraints;
    }

    public void resetEncoders() {
        wristMotor.reset();
    }

    @Override
    public void periodic() {

    }

    //Sets the elevator to target a setpoint
    public Command wristToSetpoint(double setpoint) {
        return this.run(
                () -> this.targetPosition(setpoint)).until(
                        () -> (getSetpoint().position == getGoal().position))
                .onlyIf(() -> (limitSwitch.get() == false));
    }

   // homes the elevator
   public Command home() {
    return sequence(
            this.run(() -> setWristVoltage(Volts.of(wristMotorFeedforward.calculate(-0.5))))
              .until(() -> (limitSwitch.get())),
            wristToSetpoint(1),
            waitSeconds(1),
            wristToSetpoint(0)
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
