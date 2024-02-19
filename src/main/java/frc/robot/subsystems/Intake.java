package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import frc.maps.RobotMap;
import frc.robot.Robot;

public class Intake extends SubsystemBase {
    private CCSparkMax intakeTop = new CCSparkMax("Intake Top", "IT", Constants.MotorConstants.INTAKE_TOP, MotorType.kBrushless,
            IdleMode.kBrake, Constants.MotorConstants.INTAKE_TOP_REVERSED);
    private CCSparkMax intakeBottom = new CCSparkMax("Intake Bottom", "IB", Constants.MotorConstants.INTAKE_BOTTOM,
            MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.INTAKE_BOTTOM_REVERSED);

    public Intake(){}

    public void runIntake(double speed){
        intakeBottom.set(speed);
        intakeTop.set(speed);
    }

    public Command intake(double speed){
        return this.runEnd(() -> runIntake(speed), () -> runIntake(0));
    }
    public Command halt(){
                return Commands.runOnce(()-> {}, this);
        }
}


