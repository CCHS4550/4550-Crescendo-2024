package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        shooterTop.follow(shooterBottom);
    }

    public void setShooterSpeed(double speed) {
        shooterBottom.set(speed);
    }

    public void setIndexerSpeed(double speed) {
        indexer.set(speed);
    }

    public Command shoot() {
        return this.run(() -> setShooterSpeed(1));
    }

    public Command index() {
        return this.run(() -> indexer.set(0.5));
    }

    public Command rev() {
        return this.run(() -> setShooterSpeed(1));
    }
    public Command halt(){
                return Commands.runOnce(()-> {}, this);
        }
}
