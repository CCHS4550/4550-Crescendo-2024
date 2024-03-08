package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
        private CCSparkMax intakeBack = new CCSparkMax("Intake Top", "IT", Constants.MotorConstants.INTAKE_BACK,
                        MotorType.kBrushless,
                        IdleMode.kBrake, Constants.MotorConstants.INTAKE_RIGHT_REVERSED);
        private CCSparkMax intakeFront = new CCSparkMax("Intake Bottom", "IB", Constants.MotorConstants.INTAKE_FRONT,
                        MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.INTAKE_LEFT_REVERSED);

        public Intake() {
        }

        public void setIntakeVoltage(double speed) {
                if (Math.abs(speed) <= 0.05) {
                        intakeFront.set(0);
                        intakeBack.set(0);
                } else {
                        intakeFront.setVoltage(speed * 12);
                        intakeBack.setVoltage((speed + 0.1) * 12);
                }
        }

        public Command intake(DoubleSupplier speed) {
                return this.runEnd(() -> setIntakeVoltage(speed.getAsDouble()), () -> setIntakeVoltage(0));
        }

        public Command intakeForTime(double time){
                return intake(() -> 0.4).withTimeout(time).withName("Intake For Time");
        }

        public Command halt() {
                return Commands.runOnce(() -> {
                }, this);
        }
}
