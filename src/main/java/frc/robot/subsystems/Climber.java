package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
public class Climber extends SubsystemBase{
    private CCSparkMax climberMotorRight = new CCSparkMax("Climber One", "CO", Constants.MotorConstants.CLIMBER_RIGHT, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.CLIMBER_RIGHT_REVERSED);
    private CCSparkMax climberMotorLeft = new CCSparkMax("Climber Two", "CT", Constants.MotorConstants.CLIMBER_LEFT, MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.CLIMBER_LEFT_REVERSED);
    public void upClimb() {
        climberMotorRight.set(.5);
        climberMotorLeft.set(.5);
    }
    public void downClimb() {   
        climberMotorRight.set(-.5);
        climberMotorLeft.set(-.5);
    }
    public Command halt(){
        return Commands.runOnce(()-> {}, this);
    }
}
