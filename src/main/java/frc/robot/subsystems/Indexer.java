package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.Constants;

public class Indexer extends SubsystemBase{
    private CCSparkMax indexer = new CCSparkMax("index", "in", Constants.MotorConstants.INDEXER, MotorType.kBrushless,
            IdleMode.kBrake, Constants.MotorConstants.INDEXER_REVERSED);

    public Indexer() {
    }

    public void setIndexerVoltage(double speed) {
        indexer.setVoltage(speed * 12.0);
    }

    public Command index(DoubleSupplier speed) {
        return this.runEnd(() -> setIndexerVoltage(speed.getAsDouble()), () -> setIndexerVoltage(0)).withName("Index");
    }

    public Command indexForTime(double speed, double seconds) {
        return index(() -> speed).withTimeout(seconds);
    }
}
