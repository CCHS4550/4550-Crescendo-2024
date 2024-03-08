package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.filter.SlewRateLimiter;
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


    private SlewRateLimiter shooterSlewRateLimiter = new SlewRateLimiter(0.5,-1,0);

    public Shooter() {
        // shooterTop.follow(shooterBottom);
    }

    public void setShooterVoltage(double speed) {
        shooterBottom.setVoltage(speed * 12);
        shooterTop.setVoltage(speed * 12);
    }

    public Command shoot(DoubleSupplier speed) {
        return this.runEnd(() -> setShooterVoltage(shooterSlewRateLimiter.calculate(speed.getAsDouble())), () -> setShooterVoltage(0));
    }

    
    public Command shootForTime(double speed, double seconds) {
        return shoot(() -> speed).withTimeout(seconds);
    }

    public Command rev() {
        return this.run(() -> setShooterVoltage(1));
    }
    public Command halt(){
                return Commands.runOnce(()-> {}, this);
        }
}