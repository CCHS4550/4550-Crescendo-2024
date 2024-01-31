package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;

public class Shooter extends SubsystemBase {
    private CCSparkMax shooterTop = new CCSparkMax("Shooter Top","ST", RobotMap.SHOOTER_TOP, MotorType.kBrushless, IdleMode.kCoast, RobotMap.SHOOTER_TOP_REVERSED);
    private CCSparkMax shooterBottom = new CCSparkMax("Shooter Bottom","SB", RobotMap.SHOOTER_BOTTOM, MotorType.kBrushless, IdleMode.kCoast, RobotMap.SHOOTER_BOTTOM_REVERSED);

    private CCSparkMax indexer = new CCSparkMax("index", "in", RobotMap.INDEXER, MotorType.kBrushless, IdleMode.kBrake, RobotMap.INDEXER_REVERSED);

    public Shooter(){
        shooterTop.follow(shooterBottom);
    }

    public void setShooterSpped(double speed){
        shooterBottom.set(speed);
    }

    public void setIndexerSpeed(double speed){
        indexer.set(speed);
    }

    

}
