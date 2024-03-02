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

public class Shooter extends SubsystemBase {
    private CCSparkMax shooterTop = new CCSparkMax("Shooter Top", "ST", Constants.MotorConstants.SHOOTER_TOP,
            MotorType.kBrushless, IdleMode.kCoast, Constants.MotorConstants.SHOOTER_TOP_REVERSED);
    private CCSparkMax shooterBottom = new CCSparkMax("Shooter Bottom", "SB", Constants.MotorConstants.SHOOTER_BOTTOM,
            MotorType.kBrushless, IdleMode.kCoast, Constants.MotorConstants.SHOOTER_BOTTOM_REVERSED);

    private CCSparkMax indexer = new CCSparkMax("index", "in", Constants.MotorConstants.INDEXER, MotorType.kBrushless,
            IdleMode.kBrake, Constants.MotorConstants.INDEXER_REVERSED);

    public Shooter() {
        // shooterTop.follow(shooterBottom);
    }

    public void setShooterSpeed(double speed) {
        shooterBottom.set(speed);
        shooterTop.set(speed);
    }

    public void setIndexerSpeed(double speed) {
       indexer.set(speed);
    }

    public Command shoot() {
        return this.run(() -> setShooterSpeed(0.3));
    }

    public Command index() {
        return this.runEnd(() -> setIndexerSpeed(0.5), () -> setIndexerSpeed(0)).withName("Index");
    }
    public Command indexForTime(double seconds) {
        return index().withTimeout(seconds);
    }
    
    public Command shootForTime(double seconds) {
        // WaitCommand timer = new WaitCommand(1.0);
        // return deadline(timer, shoot());
        return shoot().withTimeout(seconds);
    }

    public Command rev() {
        return this.run(() -> setShooterSpeed(1));
    }
    public Command halt(){
                return Commands.runOnce(()-> {}, this);
        }
}
