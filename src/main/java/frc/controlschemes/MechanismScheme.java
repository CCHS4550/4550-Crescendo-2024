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
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Climber;

public class MechanismScheme implements ControlScheme {
    private static CommandGenericHID buttonBoard;
    // public static CommandXboxController controller;

    public static void configure(Intake intake, Shooter shooter, Elevator elevator, Wrist wrist, Indexer indexer,
            int port) {
        buttonBoard = new CommandGenericHID(port);
        // controller = new CommandXboxController(3);
        configureButtons(port, intake, shooter, elevator, wrist, indexer);
        // arm.setDefaultCommand(Commands.run(() -> arm.moveArm(OI.axis(1,
        // ControlMap.L_JOYSTICK_VERTICAL) * 0.5,
        // OI.axis(1, ControlMap.R_JOYSTICK_VERTICAL) * 0.5), arm));

        // intake.setDefaultCommand(Commands.run(() -> intake.spin(
        // OI.axis(1, ControlMap.LT), OI.axis(1, ControlMap.RT)), intake));

        // new JoystickButton(OI.joystickArray[1], ControlMap.A_BUTTON)
        // .onTrue(Commands.run(() -> intake.toggleReverse(true),
        // intake)).onFalse(Commands.run(() -> intake.toggleReverse(false), intake));
    }

    public static void configureButtons(int port, Intake intake, Shooter shooter, Elevator elevator, Wrist wrist,
            Indexer indexer) {
        // intake.setDefaultCommand(run(() ->
        // intake.runIntake(Math.abs(controller.getLeftY()) >= 0.1 ?
        // controller.getLeftY() : 0),intake));

        // controller.axisGreaterThan(Constants.XboxConstants.RT,
        // 0.05).whileTrue(shooter.shoot())
        // .whileFalse(run(() -> shooter.setShooterSpeed(0)));
        // wrist.setDefaultCommand(wrist.setWristDutyCycle(() ->
        // MathUtil.applyDeadband(controller.getLeftY(), 0.1)));

        // controller.x().whileTrue(shooter.index());
        // controller.leftBumper().whileTrue(intake.intake(-0.7));
        // // controller.a().onTrue(wrist.targetPosition(0));
        // controller.rightBumper().whileTrue(intake.intake(0.7));
        // controller.b().onTrue(elevator.home());
        // controller.y().onTrue(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_TOP));
        // elevator.setDefaultCommand(
        Command ampScore = sequence(
                parallel(sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL)),
                        elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP)),
                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP),
                parallel(indexer.indexForTime(0.3, 0.3), shooter.shootForTime(0.3, 1))).withName("Amp Score");

        Command autoShoot = sequence(parallel(shooter.shootForTime(-0.1, 0.1), indexer.indexForTime(-0.1, 0.2)),
                shooter.shootForTime(1, 1.5),
                parallel(shooter.shootForTime(1, 0.3), indexer.indexForTime(0.3, 0.3))).withName("Auto Shoot");

        Command subWooferShoot = sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT), autoShoot).withName("Subwoofer Shoot");

        // this is the good stuff
        buttonBoard.button(1).whileTrue(elevator.setElevatorDutyCycle(() -> 0.3));
        buttonBoard.button(2).whileTrue(elevator.setElevatorDutyCycle(() -> -0.3));
        buttonBoard.button(3).whileTrue(wrist.setWristDutyCycle(() -> 0.3));
        buttonBoard.button(4).whileTrue(wrist.setWristDutyCycle(() -> -0.3));
        buttonBoard.button(5).onTrue(sequence(elevator.home(), wrist.home()));


        buttonBoard.button(9)
                .whileTrue((sequence(parallel(intake.intake(() -> -1), indexer.index(() -> 0.3)))));

        // amp
        buttonBoard.button(10)
                .onTrue(ampScore);

        // buttonBoard.button(12).onTrue(sequence(parallel(sequence(wrist.wristToSetpoint(15.0),shooter.indexForTime(-0.1,
        // 0.3)), elevator.elevatorToSetpoint(75)),
        // wrist.wristToSetpoint(43)));

        buttonBoard.button(12)
                .onTrue(sequence(
                        parallel(sequence(wrist.wristToSetpoint(15.0), indexer.indexForTime(-0.1, 0.3)),
                                elevator.elevatorToSetpoint(75)),
                        parallel(wrist.wristToSetpoint(22.4), shooter.shootForTime(1, 2)),
                        indexer.indexForTime(0.3, 0.3)));

        // intake.setDefaultCommand(intake.intake(() ->
        // MathUtil.applyDeadband(controller.getLeftY(), 0.1)));

        Command targetAmp = sequence(
                parallel(sequence(wrist.wristToSetpoint(15.0), indexer.indexForTime(-0.1, 0.3)),
                        elevator.elevatorToSetpoint(75)),
                parallel(wrist.wristToSetpoint(22.4), shooter.shootForTime(1, 2)),
                indexer.indexForTime(0.3, 0.3));

        // buttonBoard.button(2).onTrue(parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_SHOOT),
        // wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT)).withName("Target
        // Shoot"))

    }

}
