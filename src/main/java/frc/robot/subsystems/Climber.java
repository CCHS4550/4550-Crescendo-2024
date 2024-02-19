package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;
import frc.maps.Constants.MotorConstants;

import static frc.maps.Constants.*;

public class Climber extends SubsystemBase {
    private CCSparkMax climberLeft = new CCSparkMax("Climber Left", "CL", Constants.MotorConstants.CLIMBER_LEFT,
            MotorType.kBrushless, IdleMode.kBrake, MotorConstants.CLIMBER_LEFT_REVERSED);
    private CCSparkMax climberRight = new CCSparkMax("Climber Right", "CR", Constants.MotorConstants.CLIMBER_RIGHT,
            MotorType.kBrushless, IdleMode.kBrake, MotorConstants.CLIMBER_RIGHT_REVERSED);

    public Climber() {

    }

    public void runClimberLeft(double speed) {
        climberLeft.set(speed);
    }

    public void runClimberRight(double speed) {
        climberRight.set(speed);
    }

    public void runClimber(double speed) {
        runClimberLeft(speed);
        runClimberRight(speed);
    }
    public Command climb(){
        return this.run(() -> runClimber(0.5));
    }
    public Command halt(){
                return Commands.runOnce(()-> {}, this);
        }
}
