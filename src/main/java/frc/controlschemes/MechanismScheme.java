package frc.controlschemes;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.helpers.ControlScheme;
import frc.maps.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Climber;

public class MechanismScheme implements ControlScheme {
    private static CommandGenericHID buttonBoard;
    public static CommandXboxController controller;

    public static void configure(Intake intake, Shooter shooter, Elevator elevator, Wrist wrist, int port) {
        // buttonBoard = new CommandGenericHID(port);
        controller = new CommandXboxController(port);
        configureButtons(port, intake, shooter, elevator, wrist);
        // arm.setDefaultCommand(Commands.run(() -> arm.moveArm(OI.axis(1,
        // ControlMap.L_JOYSTICK_VERTICAL) * 0.5,
        // OI.axis(1, ControlMap.R_JOYSTICK_VERTICAL) * 0.5), arm));

        // intake.setDefaultCommand(Commands.run(() -> intake.spin(
        // OI.axis(1, ControlMap.LT), OI.axis(1, ControlMap.RT)), intake));

        // new JoystickButton(OI.joystickArray[1], ControlMap.A_BUTTON)
        // .onTrue(Commands.run(() -> intake.toggleReverse(true),
        // intake)).onFalse(Commands.run(() -> intake.toggleReverse(false), intake));
    }

    public static void configureButtons(int port, Intake intake, Shooter shooter, Elevator elevator, Wrist wrist) {
        // intake.setDefaultCommand(run(() ->
        // intake.runIntake(Math.abs(controller.getLeftY()) >= 0.1 ?
        // controller.getLeftY() : 0),intake));

        controller.axisGreaterThan(Constants.XboxConstants.RT, 0.05).whileTrue(shooter.shoot())
                .whileFalse(run(() -> shooter.setShooterSpeed(0)));
        wrist.setDefaultCommand(wrist.setWristDutyCycle(() -> MathUtil.applyDeadband(controller.getLeftY(), 0.05)));

        controller.x().whileTrue(shooter.index());
        controller.a().whileTrue(intake.intake(-0.4));
        // controller.b().whileTrue(intake.intake(0.4));
        controller.b().onTrue(elevator.home());
        controller.y().onTrue(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_TOP));
        elevator.setDefaultCommand(
                elevator.setElevatorDutyCycle(() -> MathUtil.applyDeadband(-controller.getRightY(), 0.09)));
        // elevator.setDefaultCommand( elevator.runElevatorVoltageSpeed(() ->
        // MathUtil.applyDeadband(-controller.getRightY(), 0.09) * 77));
        // SequentialCommandGroup autoShoot = (SequentialCommandGroup) sequence(
        // wrist.wristToSetpoint(wrist.autoWristAngle(swerveDrive, elevator)),
        // shooter.rev(), shooter.indexOneSecond());
        // buttonBoard.button(1).onTrue(parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_INTAKE),
        // wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_INTAKE)).withName("Target
        // Intake"));

        // buttonBoard.button(2).onTrue(parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_SHOOT),
        // wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT)).withName("Target
        // Shoot"));

        // buttonBoard.button(3).onTrue(parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP),
        // wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP)).withName("Target
        // Amp"));
        // buttonBoard.button(4).whileTrue(intake.intake(0.5));
        // buttonBoard.button(5).whileTrue(shooter.index());
        // buttonBoard.button(6).toggleOnTrue(shooter.rev());
        // buttonBoard.button(7).whileTrue(shooter.shoot());
        // buttonBoard.button(8).whileTrue(climber.climb());
        // buttonBoard.button(9).onTrue(autoShoot);
        // buttonBoard.button(11)
        // .onTrue(parallel(intake.halt(), shooter.halt(), elevator.halt(),
        // wrist.halt(), climber.halt()));

    }

}
