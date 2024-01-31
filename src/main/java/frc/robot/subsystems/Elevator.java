package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;

public class Elevator extends SubsystemBase {
    private CCSparkMax elevatorMotorOne = new CCSparkMax("Elevator One", "EO", RobotMap.ELEVATOR_ONE,
            MotorType.kBrushless, IdleMode.kBrake, RobotMap.ELEVATOR_ONE_REVERSED);
    private CCSparkMax elevatorMotorTwo = new CCSparkMax("Elevator Two", "ET", RobotMap.ELEVATOR_TWO,
            MotorType.kBrushless, IdleMode.kBrake, RobotMap.ELEVATOR_TWO_REVERSED);

    private SimpleMotorFeedforward elevatorMotorFeedforward;

    private PIDController elevatorPidController;

    private TrapezoidProfile.Constraints constraints;

    private TrapezoidProfile profile;

    private TrapezoidProfile.State setPoint, goal;

    public Elevator(){
        elevatorMotorFeedforward = new SimpleMotorFeedforward(RobotMap.ELEVATOR_KS, RobotMap.ELEVATOR_KV, RobotMap.ELEVATOR_KA);
       
        constraints = new Constraints(MetersPerSecond.of(1), MetersPerSecondPerSecond.of(0.5));
        profile = new TrapezoidProfile(constraints);
        setPoint = new TrapezoidProfile.State();
        goal = new TrapezoidProfile.State();
    }

    public void setElevatorVoltage(Measure<Voltage> volts){
        elevatorMotorOne.setVoltage(volts.magnitude());
         elevatorMotorTwo.setVoltage(volts.magnitude());
    }

    public void setSetpoint(TrapezoidProfile.State setPoint){
        this.setPoint = setPoint;
    }

    public TrapezoidProfile.State getSetpoint(){
        return setPoint;
    }

    public void setGoal(double goalState){
        goal = new TrapezoidProfile.State(goalState,0);
    }

    public State getGoal(){
        return goal;
    }

    public double getElevatorPosition() {
        return edu.wpi.first.math.util.Units.rotationsToRadians(elevatorMotorOne.getPosition());
      }

    public TrapezoidProfile.Constraints getConstraints(){
        return constraints;
    }

    public void resetEncoders(){
        elevatorMotorOne.reset();
        elevatorMotorTwo.reset();
    }
}
