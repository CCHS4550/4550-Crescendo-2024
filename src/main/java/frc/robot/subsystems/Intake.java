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
        private CCSparkMax intakeBack = new CCSparkMax("Intake Top", "IT", Constants.MotorConstants.INTAKE_BACK,
                        MotorType.kBrushless,
                        IdleMode.kBrake, Constants.MotorConstants.INTAKE_RIGHT_REVERSED);
        private CCSparkMax intakeFront = new CCSparkMax("Intake Bottom", "IB", Constants.MotorConstants.INTAKE_FRONT,
                        MotorType.kBrushless, IdleMode.kBrake, Constants.MotorConstants.INTAKE_LEFT_REVERSED);

        public Intake() {
        }

        public void runIntake(double speed) {
                if (Math.abs(speed) <= 0.05) {
                        intakeFront.set(0);
                        intakeBack.set(0);
                } else {
                        intakeFront.set(speed);
                        intakeBack.set(speed + 0.1);
                }
        }

        public Command intake(double speed) {
                return this.runEnd(() -> runIntake(speed), () -> runIntake(0));
        }

        public Command halt() {
                return Commands.runOnce(() -> {
                }, this);
        }
}
