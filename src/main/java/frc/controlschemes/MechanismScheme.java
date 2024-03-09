package frc.controlschemes;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.helpers.ControlScheme;
import frc.maps.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;

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
                Command ampScore = sequence(
                                parallel(sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL)),
                                                elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP)),
                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP),
                                parallel(indexer.indexForTime(0.3, 0.3), shooter.shootForTime(0.3, 1)))
                                .withName("Amp Score");

                Command autoShoot = sequence(parallel(shooter.shootForTime(-0.1, 0.1), indexer.indexForTime(-0.1, 0.2)),
                                shooter.shootForTime(1, 1.5),
                                parallel(shooter.shootForTime(1, 0.3), indexer.indexForTime(0.3, 0.3)))
                                .withName("Auto Shoot");

                // Command targetSubWooferShoot =
                // sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT),
                // autoShoot).withName("Subwoofer Shoot");

                Command targetAmp = sequence(
                                parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP),
                                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL)),
                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP));

                Command ampScoreFull = sequence(
                                parallel(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL)
                                                .withTimeout(1),
                                                elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP)
                                                                .withTimeout(2.5)),
                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP).withTimeout(2),
                                parallel(indexer.indexForTime(0.3, 0.6)),
                                shooter.shootForTime(0.4, 0.6),
                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL).withTimeout(1.5),
                                parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_INTAKE),
                                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_INTAKE))
                                                .withTimeout(2.5),
                                parallel(elevator.home(), wrist.home()));

                Command targetShoot = parallel(elevator.elevatorToSetpoint(
                                Constants.MechanismPositions.ELEVATOR_SHOOT),
                                wrist.wristToSetpoint(
                                                Constants.MechanismPositions.WRIST_SHOOT));

                // this is the good stuff
                buttonBoard.button(1)
                                .whileTrue(((parallel(intake.intake(() -> -1), indexer.index(() -> 0.3),
                                                shooter.shoot(() -> -.1)))));
                buttonBoard.button(2).onTrue(sequence(elevator.home(), wrist.home()));
                buttonBoard.button(3).onTrue(targetShoot);
                buttonBoard.button(4).whileTrue(parallel(shooter.shoot(() -> -0.1), indexer.index(() -> -0.1),
                                intake.intake(() -> 0.8)));
                buttonBoard.button(5).onTrue(ampScoreFull);
                // buttonBoard.button(6).onTrue(autoShoot);
                buttonBoard.button(6).onTrue(parallel(elevator.elevatorToSetpoint(45),
                                wrist.wristToSetpoint(34.54)));
                buttonBoard.button(7).whileTrue(elevator.setElevatorDutyCycle(() -> 0.3));
                buttonBoard.button(8).whileTrue(elevator.setElevatorDutyCycle(() -> -0.3));
                buttonBoard.button(9).whileTrue(wrist.setWristDutyCycle(() -> 0.3));
                buttonBoard.button(10).whileTrue(wrist.setWristDutyCycle(() -> -0.3));
                buttonBoard.button(11).whileTrue(sequence(indexer.indexForTime(-0.2, 0.1), shooter.shoot(() -> 0.7)));
                buttonBoard.button(12).onTrue(indexer.indexForTime(0.3, 0.5));
                // buttonBoard.button(12).onTrue(autoShoot);
                // amp
                // buttonBoard.button(10)
                // .onTrue(ampScore);

        }

}
