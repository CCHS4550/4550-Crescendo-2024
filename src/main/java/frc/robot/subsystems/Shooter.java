package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.deadline;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
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


    private SlewRateLimiter shooterSlewRateLimiter = new SlewRateLimiter(0.5,-1,0);

    public Shooter() {
        // shooterTop.follow(shooterBottom);
    }

    public void setShooterSpeed(double speed) {
        shooterBottom.set(speed);
        shooterTop.set(speed);
    }

    public Command shoot(DoubleSupplier speed) {
        return this.runEnd(() -> setShooterSpeed(shooterSlewRateLimiter.calculate(speed.getAsDouble())), () -> setShooterSpeed(0));
    }

    
    public Command shootForTime(double speed, double seconds) {
        return shoot(() -> speed).withTimeout(seconds);
    }

    public Command rev() {
        return this.run(() -> setShooterSpeed(1));
    }
    public Command halt(){
                return Commands.runOnce(()-> {}, this);
        }
}
