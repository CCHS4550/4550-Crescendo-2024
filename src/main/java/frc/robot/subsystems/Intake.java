package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        private SlewRateLimiter intakRateLimiter = new SlewRateLimiter(0.5, -10000000.0, 0);
        public Intake() {
        }

        public void setIntakeVoltage(DoubleSupplier speed) {
                double rateLimitedSpeed = intakRateLimiter.calculate(speed.getAsDouble());
                SmartDashboard.putNumber("Speed", rateLimitedSpeed);
                if (Math.abs(speed.getAsDouble()) <= 0.05) {
                        intakeFront.set(0);
                        intakeBack.set(0);
                } else {
                        intakeFront.setVoltage(rateLimitedSpeed * 12);
                        intakeBack.setVoltage((rateLimitedSpeed + 0.1) * 12);
                }
        }

        public Command intake(DoubleSupplier speed) {
                return this.runEnd(() -> setIntakeVoltage(speed), () -> setIntakeVoltage(() -> 0));
        }

        public Command intakeForTime(double time){
                return intake(() -> 0.4).withTimeout(time).withName("Intake For Time");
        }

        public Command halt() {
                return Commands.runOnce(() -> {
                }, this);
        }
}
