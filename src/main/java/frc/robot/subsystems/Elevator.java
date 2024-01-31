package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;
import com.revrobotics.SparkPIDController.ArbFFUnits;

public class Elevator {
    private CCSparkMax elevatorMotorOne = new CCSparkMax("Elevator One", "EO", RobotMap.ELEVATOR_ONE,
            MotorType.kBrushless, IdleMode.kBrake, RobotMap.ELEVATOR_ONE_REVERSED);
    private CCSparkMax elevatorMotorTwo = new CCSparkMax("Elevator Two", "ET", RobotMap.ELEVATOR_TWO,
            MotorType.kBrushless, IdleMode.kBrake, RobotMap.ELEVATOR_TWO_REVERSED);

    private SimpleMotorFeedforward elevatorMotorFeedforward;

    private PIDController elevatorPidController;

    public Elevator(){
        elevatorMotorFeedforward = new SimpleMotorFeedforward(RobotMap.ELEVATOR_KS, RobotMap.ELEVATOR_KV, RobotMap.ELEVATOR_KA);
    }

    public void setPosition(double pos){
        double volts = 0;

        volts = elevatorMotorFeedforward.calculate(?);

        elevatorMotorOne.getPIDController().setReference(pos, ControlType.kPosition, 0, volts, ArbFFUnits.kVoltage);
        elevatorMotorTwo.getPIDController().setReference(pos, ControlType.kPosition, 0, volts, ArbFFUnits.kVoltage);
        
        elevatorMotorOne.setVoltage(volts);
        elevatorMotorTwo.setVoltage(volts);
    }



    public void resetEncoders(){
        elevatorMotorOne.reset();
        elevatorMotorTwo.reset();
    }
}
