package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import frc.maps.Constants.BlueFieldPositionConstants;
import frc.maps.Constants.RedFieldPositionConstants;
import frc.maps.RobotMap;

public class Wrist extends SubsystemBase {

    SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(1), Volts.of(5), Seconds.of(4),
                    (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                    (voltage) -> setWristVoltage(voltage),
                    null, // No log consumer, since data is recorded by URCL
                    this));

    private CCSparkMax wristMotor = new CCSparkMax("Wrist Motor", "WM", Constants.MotorConstants.WRIST,
            MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.WRIST_REVERSED);

    private ElevatorFeedforward wristMotorFeedforward;

    private PIDController wristPidController;

    private TrapezoidProfile.Constraints constraints;

    private TrapezoidProfile profile;

    private TrapezoidProfile.State setPoint, goal;

    private GenericEntry wristPositionEntry;

    // private DigitalInput elevatorBottom = new DigitalInput(1);

    private DigitalInput wristSwitch = new DigitalInput(3);

    public Wrist() {
        wristMotorFeedforward = new ElevatorFeedforward(
                Constants.FeedForwardConstants.WRIST_KS,
                Constants.FeedForwardConstants.WRIST_KG,
                Constants.FeedForwardConstants.WRIST_KV,
                Constants.FeedForwardConstants.WRIST_KA);
        wristPidController = new PIDController(1.2, 0, 0);

        constraints = new Constraints(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.5));
        profile = new TrapezoidProfile(constraints);
        setPoint = new TrapezoidProfile.State();
        goal = new TrapezoidProfile.State();

        wristPositionEntry = Shuffleboard.getTab("Encoders").add("Wrist Position", wristMotor.getPosition()).getEntry();
    }

    public void targetPosition(double position) {
        setGoal(position);
        TrapezoidProfile.State nextSetpoint = profile.calculate(0.02, getSetpoint(), getGoal());
        double feedForwardPower = wristMotorFeedforward.calculate(nextSetpoint.velocity);
        setSetpoint(nextSetpoint);
        Logger.recordOutput("Wrist/Trapezoid Position", nextSetpoint.position);
        double pidCalc = wristPidController.calculate(wristMotor.getPosition(), position);
        wristMotor.setVoltage(feedForwardPower + pidCalc);

        // double elevatorPower = elevatorPidController.calculate(currentPosition);

        // wristMotor.getPIDController().setReference(position, ControlType.kPosition,
        // 0, feedForwardPower,
        // ArbFFUnits.kVoltage);
    }

    /**
     * Sets the elevator to target a setpoint
     */
    public Command wristToSetpoint(double setpoint) {
        return this.runEnd(
                () -> this.targetPosition(setpoint), () -> setWristVoltage(Volts.of(0))).until(
                        () -> (Math.abs(wristMotor.getPosition() - setpoint) <= 0.3));
    }

    public Command home() {
        return setWristDutyCycle(() -> -0.4);
    }

    public double autoWristAngle(SwerveDrive swerveDrive, Elevator elevator) {
        Pose2d robotPose = swerveDrive.getPose();
        Pose2d speakerPose = new Pose2d();
        double height = 78 - elevator.elevatorElevation();
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            speakerPose = RedFieldPositionConstants.SPEAKER_FRONT;
        } else {
            speakerPose = BlueFieldPositionConstants.SPEAKER_FRONT;
        }
        double distanceToSpeaker = PhotonUtils.getDistanceToPose(robotPose, speakerPose);
        double wristAngle = Math.atan(height / distanceToSpeaker);
        return wristAngle;
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

    public double getWristPositionRadians() {
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
        wristPositionEntry.setDouble(wristMotor.getPosition());
        SmartDashboard.putBoolean("Wrist Switch", wristSwitch.get());
        if (wristSwitch.get()) {
            resetEncoders();
        }
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

    public Command setWristDutyCycle(DoubleSupplier speed) {
        return this.runEnd(() -> wristMotor.set(speed.getAsDouble()), () -> wristMotor.set(0))
                .until(() -> wristSwitch.get() && speed.getAsDouble() < 0);
    }

    public Command setWristVoltageCycle(DoubleSupplier speed) {
        return this.run(() -> wristMotor.setVoltage(wristMotorFeedforward.calculate(speed.getAsDouble())))
                .until(() -> wristSwitch.get() && speed.getAsDouble() > 0);
    }
}
