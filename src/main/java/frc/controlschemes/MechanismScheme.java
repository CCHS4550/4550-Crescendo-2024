package frc.controlschemes;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.function.BooleanSupplier;

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
        public static BooleanSupplier inAmpPosition = () -> false;

        public static Command ampShootCurrentCommand = null;
        public static boolean runAmpShoot = false;
        public static Command shootCmd = null;
        public static Command ampCmd = null;
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
                // intake)).onFalse(Commands.run(() -> intake.toggleReverse(false), intake));\
        }

        public static void configureButtons(int port, Intake intake, Shooter shooter, Elevator elevator, Wrist wrist,
                        Indexer indexer) {

                Command ampScore = sequence
                                (
                                parallel(
                                        wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL),
                                        elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP)
                                        ),
                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP),
                                parallel(
                                indexer.indexForTime(0.3, 0.3),
                                shooter.shootForTime(0.3, 1)
                                        )
                                )
                                .withName("Amp Score");

                Command autoShoot = sequence
                                (
                                parallel(
                                        shooter.shootForTime(-0.1, 0.1),
                                        indexer.indexForTime(-0.1, 0.2)
                                        ),
                                shooter.shootForTime(1, 1.5),
                                parallel(
                                        shooter.shootForTime(1, 0.3),
                                        indexer.indexForTime(0.3, 0.3)
                                        )
                                )
                                .withName("Auto Shoot");

                // Command targetSubWooferShoot =
                // sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_SHOOT),
                // autoShoot).withName("Subwoofer Shoot");

                Command shoot = indexer.indexForTime(0.3, 0.5);
                shootCmd = shoot;

                Command targetAmp = sequence(
                                parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP)
                                                .withTimeout(2.5),
                                        wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL)
                                                .withTimeout(1),
                                        setInAmpPosition(true)));
                                // wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP));

                Command finishAmpSequence = sequence(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP).withTimeout(.75),
                                parallel
                                (indexer.indexForTime(0.3, 0.6),
                                shooter.shootForTime(0.4, 0.6)),
                                waitSeconds(0.25),
                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL).withTimeout(.25),
                                parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_INTAKE),
                                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_INTAKE))
                                                .withTimeout(2.5),
                                parallel(elevator.home(), wrist.home()), setInAmpPosition(false));
                
                ampCmd  = finishAmpSequence;

                Command ampScoreFull = sequence(
                                parallel(wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL)
                                                .withTimeout(1),
                                                elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_AMP)
                                                                .withTimeout(2.5)),
                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_AMP).withTimeout(2),
                                parallel(indexer.indexForTime(0.3, 0.6),
                                shooter.shootForTime(0.4, 0.6)),
                                waitSeconds(0.5),
                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_TRAVEL).withTimeout(1.5),
                                parallel(elevator.elevatorToSetpoint(Constants.MechanismPositions.ELEVATOR_INTAKE),
                                                wrist.wristToSetpoint(Constants.MechanismPositions.WRIST_INTAKE))
                                                .withTimeout(2.5),
                                parallel(elevator.home(), wrist.home()));
                                
                Command targetShoot = parallel(elevator.elevatorToSetpoint(
                                Constants.MechanismPositions.ELEVATOR_SHOOT),
                                wrist.wristToSetpoint(
                                                Constants.MechanismPositions.WRIST_SHOOT),
                                setInAmpPosition(false));
//Constants.MechanismPositions.WRIST_SHOOT
                ampShootCurrentCommand = shootCmd;



                // this is the good stuff
                /* Intake */
                buttonBoard.button(1)
                                .whileTrue(((parallel(
                                        intake.intake(() -> -1), 
                                        indexer.index(() -> 0.3),
                                        shooter.shoot(() -> -.1), 
                                        wrist.wristToSetpoint(0.0), 
                                        elevator.elevatorToSetpoint(0)))));
                /* Home */
                buttonBoard.button(2).onTrue(parallel(sequence(elevator.home(), wrist.home()), setInAmpPosition(false)));
                /* Shoot Position */
                buttonBoard.button(3).onTrue(targetShoot);
                /* Stop! */
                buttonBoard.button(4).whileTrue(parallel(shooter.shoot(() -> -0.1), indexer.index(() -> -0.1),
                                intake.intake(() -> 0.8)));
                // buttonBoard.button(5).onTrue(ampScoreFull);
                /* Amp position */
                buttonBoard.button(5).onTrue(targetAmp);
                // buttonBoard.button(6).onTrue(autoShoot);
                // Climbing
                buttonBoard.button(6).onTrue(parallel(elevator.elevatorToSetpoint(45),
                                wrist.wristToSetpoint(34.54),
                                setInAmpPosition(false)));
                /* Elevator Manual Up */
                buttonBoard.button(7).whileTrue(parallel(elevator.setElevatorDutyCycle(() -> 0.3), setInAmpPosition(false)));
                /* Elevator Manual Down */
                buttonBoard.button(8).whileTrue(parallel(elevator.setElevatorDutyCycle(() -> -0.3), setInAmpPosition(false)));
                /* Wrist Manual Up */
                buttonBoard.button(9).whileTrue(parallel(wrist.setWristDutyCycle(() -> 0.3), setInAmpPosition(false)));
                /* Wrist Manual Down */
                buttonBoard.button(10).whileTrue(parallel(wrist.setWristDutyCycle(() -> -0.3), setInAmpPosition(false)));
                /* Rev */
                buttonBoard.button(11).whileTrue(sequence(indexer.indexForTime(-0.2, 0.1), shooter.shoot(() -> 0.7)));
                // buttonBoard.button(12).onTrue(indexer.indexForTime(0.3, 0.5));
                /* Shoot! */
                buttonBoard.button(12).onTrue(runOnce(() -> runAmpShoot()));
                // buttonBoard.button(12).onTrue(autoShoot);
                // buttonBoard.button(12).onTrue(finishAmpSequence);
                // amp
                // buttonBoard.button(10)
                // .onTrue(ampScore);
        }

        public static void periodic() {
                ampShootCurrentCommand = inAmpPosition.getAsBoolean() ? ampCmd : shootCmd;
                if(runAmpShoot) {
                        ampShootCurrentCommand.schedule();
                        runAmpShoot = false;
                }

        }

        private static Command setInAmpPosition(boolean inPos) {
                return runOnce(() -> inAmpPosition = () -> inPos);
        }

        private static void runAmpShoot () {
                runAmpShoot = true;
        }
}
