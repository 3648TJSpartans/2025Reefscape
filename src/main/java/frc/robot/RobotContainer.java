// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.commands.goToCommands.DriveToNearestIntake;
import frc.robot.commands.goToCommands.DriveToPose;
import frc.robot.commands.sftCommands.SftAnalogCmd;
import frc.robot.commands.sftCommands.SftCmd;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants.AutonState;
import frc.robot.commands.ledCommands.autonoumousIndicator;
import frc.robot.commands.ledCommands.teleopStatesIndicators;
import frc.robot.commands.autonCommands.AlgaeRemovalCmd;
import frc.robot.commands.autonCommands.AutoBuildingBlocks;
import frc.robot.commands.autonCommands.CoralSequentialAutoCmd;
import frc.robot.commands.autonCommands.CoralSequentialCmd;
import frc.robot.commands.autonCommands.SourceSequentialCmd;
import frc.robot.commands.goToCommands.DriveToNearest;
import frc.robot.commands.goToCommands.DriveToPose;
import frc.robot.commands.AlignCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.sft.Sft;
import frc.robot.subsystems.sft.SftConstants;
import frc.robot.subsystems.sft.SftIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableNumber;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeIO;
import frc.robot.subsystems.coralIntake.CoralIntakeIOSparkMax;
import frc.robot.subsystems.leds.*;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.commands.coralCommands.CoralCmd;
import frc.robot.commands.coralCommands.CoralDefaultCmd;
import frc.robot.commands.climberCommands.*;
// import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntake;
// import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntakeIO;
// import frc.robot.subsystems.coralSubsystems.coralIntake.CoralIntakeIOSparkMax;
// import frc.robot.subsystems.coralSubsystems.elevator.Elevator;
// import frc.robot.subsystems.coralSubsystems.CoralConstants;
// import frc.robot.subsystems.coralSubsystems.elevator.ElevatorIO;
// import frc.robot.subsystems.coralSubsystems.elevator.ElevatorIOSparkMax;
import frc.robot.commands.coralCommands.CoralElevatorIntegratedCmd;
import frc.robot.commands.coralCommands.CoralInCmd;
import frc.robot.commands.coralCommands.CoralOutCmd;
import frc.robot.commands.coralCommands.DownToIntakeCmd;
import frc.robot.commands.coralCommands.ElevatorCmd;
import frc.robot.commands.coralCommands.HomeElevatorCmd;
import frc.robot.commands.coralCommands.SlamCoralCmd;
import frc.robot.commands.coralCommands.UpFromIntakeCmd;
import frc.robot.commands.coralCommands.ElevatorAnalogCmd;
import frc.robot.commands.coralCommands.WristCmd;
import frc.robot.commands.coralCommands.WristAnalogCmd;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // Subsystems
        private final Drive m_drive;
        private final Vision m_vision;
        private final CoralIntake m_coral;
        private final Elevator m_elevator;
        private final LedSubsystem m_led;
        private ClimberSubsystem m_climber;
        private boolean override;
        private Sft m_sft;
        private boolean endgameClosed = true;

        // Controller
        private final CommandXboxController m_driveController = new CommandXboxController(0);
        private final CommandXboxController m_copilotController = new CommandXboxController(1);
        private final CommandXboxController m_controllerTwo = new CommandXboxController(2);
        private final CommandXboxController m_ledController = new CommandXboxController(3);
        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        // Alerts
        private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #1",
                        30.0);
        private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #2",
                        15.0);
        private final LoggedNetworkNumber endgameAlert3 = new LoggedNetworkNumber("/SmartDashboard/Endgame Alert #3",
                        5.0);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */

        public RobotContainer() {
                m_led = new LedSubsystem();
                Logger.recordOutput("Poses/shouldFlip", AllianceFlipUtil.shouldFlip());
                Logger.recordOutput("Override", override);
                override = false;
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                m_climber = new ClimberSubsystem(new ClimberIOSparkMax());
                                m_sft = new Sft(new SftIOSparkMax());
                                m_drive = new Drive(
                                                new GyroIONavX(),
                                                new ModuleIOSpark(0),
                                                new ModuleIOSpark(1),
                                                new ModuleIOSpark(2),
                                                new ModuleIOSpark(3));
                                m_vision = new Vision(
                                                m_drive::addVisionMeasurement,
                                                new VisionIOLimelight("limelight-three", m_drive::getRotation),
                                                new VisionIOLimelight("limelight-twoplus", m_drive::getRotation));
                                m_coral = new CoralIntake(new CoralIntakeIOSparkMax());
                                m_elevator = new Elevator(new ElevatorIOSparkMax());
                                break;

                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations

                                m_drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim(),
                                                new ModuleIOSim());
                                m_vision = new Vision(
                                                m_drive::addVisionMeasurement,
                                                // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0,
                                                // m_drive::getPose),
                                                new VisionIOPhotonVisionSim(VisionConstants.camera1Name,
                                                                VisionConstants.robotToCamera1,
                                                                m_drive::getPose));
                                // To later be replaced with CoralIntakeIOSim
                                m_coral = new CoralIntake(new CoralIntakeIO() {
                                });
                                m_elevator = new Elevator(new ElevatorIO() {

                                });
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                m_drive = new Drive(
                                                new GyroIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                },
                                                new ModuleIO() {
                                                });
                                m_vision = new Vision(m_drive::addVisionMeasurement, new VisionIO() {
                                }, new VisionIO() {
                                });
                                m_coral = new CoralIntake(new CoralIntakeIO() {
                                });
                                m_elevator = new Elevator(new ElevatorIO() {

                                });
                                break;
                }

                Command homeElevator = new HomeElevatorCmd(m_elevator, m_coral);
                Command coralOut = new CoralOutCmd(m_coral);
                Command l1 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                new TunableNumber("Elevator/Height/L1", ElevatorConstants.coralLeveL1).get(),
                                new TunableNumber("Elevator/Angle/L1", CoralIntakeConstants.L1Angle).get());
                Command l2 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                new TunableNumber("Elevator/Height/L2", ElevatorConstants.coralLeveL2).get(),
                                new TunableNumber("Elevator/Angle/L2", CoralIntakeConstants.L2Angle).get());
                Command l3 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                new TunableNumber("Elevator/Height/L3", ElevatorConstants.coralLeveL3).get(),
                                new TunableNumber("Elevator/Angle/L3", CoralIntakeConstants.L3Angle).get());
                Command l4 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                new TunableNumber("Elevator/Height/L4", ElevatorConstants.coralLeveL4).get(),
                                new TunableNumber("Elevator/Angle/L4", CoralIntakeConstants.L4Angle).get());
                Command intakePos = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                ElevatorConstants.intakePose, CoralIntakeConstants.IntakeAngle);
                Command downToIntake = new DownToIntakeCmd(m_coral, m_elevator);
                Command upFromIntake = new UpFromIntakeCmd(m_coral, m_elevator);
                Command coralSequential = new CoralSequentialAutoCmd(m_drive, m_coral, m_elevator, true);

                Command coralIn = new CoralInCmd(m_coral, m_elevator);

                Command slamCoral = new SlamCoralCmd(m_coral);
                NamedCommands.registerCommand("homeElevator", homeElevator);
                NamedCommands.registerCommand("l4", l4);
                NamedCommands.registerCommand("l3", l3);
                NamedCommands.registerCommand("l2", l2);

                NamedCommands.registerCommand("intakePos", intakePos);
                NamedCommands.registerCommand("intake", coralIn);
                NamedCommands.registerCommand("slamCoral", coralOut);
                NamedCommands.registerCommand("downToIntake", downToIntake);
                NamedCommands.registerCommand("upFromIntake", upFromIntake);
                NamedCommands.registerCommand("setL4", new InstantCommand(() -> CoralSequentialCmd.setLevel(4)));
                NamedCommands.registerCommand("setL3", new InstantCommand(() -> CoralSequentialCmd.setLevel(3)));
                NamedCommands.registerCommand("setL2", new InstantCommand(() -> CoralSequentialCmd.setLevel(2)));
                NamedCommands.registerCommand("setL1", new InstantCommand(() -> CoralSequentialCmd.setLevel(1)));
                NamedCommands.registerCommand("setLeft", new InstantCommand(
                                () -> CoralSequentialCmd.setAutonState(AutonState.LEFTREEF)));
                NamedCommands.registerCommand("setRight", new InstantCommand(
                                () -> CoralSequentialCmd.setAutonState(AutonState.LEFTREEF)));
                NamedCommands.registerCommand("driveToNearest", coralSequential);
                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
                configureAutoChooser();
                // Configure the button bindings
                configureButtonBindings();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // configureAutos();

                // configureLeds();
                configureClimber();
                configureCoralIntake();
                configureDrive();
                configureElevator();
                configureSetpoints();
                configureEndgameTriggers();
                configureSft();
                m_copilotController.rightTrigger().onTrue(new InstantCommand(() -> toggleOverride()));
                /*
                 * m_led.setLedPattern(LedConstants.elevatorHeight, m_led.elevatorBuffer);
                 * m_led.setLedPattern(LedConstants.teal, m_led.leftGuideBuffer);
                 * m_led.setLedPattern(LedConstants.yellow, m_led.rightGuideBuffer);
                 */
        }

        public void configureEndgameTriggers() {
                configureAlerts();
                System.out.println(Math.abs(m_drive.getPose().getX() - PoseConstants.fieldLength / 2) < 1.5);
                // When in the closed position
                Command sftClosedCmd = new CoralElevatorIntegratedCmd(m_coral, m_elevator, 0,
                                CoralIntakeConstants.endgameAngle)
                                .andThen(new WaitCommand(.75))
                                .deadlineFor(new SequentialCommandGroup(
                                                // new WaitCommand(.5),
                                                new SftCmd(m_sft, 0))
                                                .alongWith(new InstantCommand(
                                                                () -> System.out.println("sftCLOSEDRan"))));

                Command sftOpen = new CoralElevatorIntegratedCmd(m_coral, m_elevator, 0,
                                CoralIntakeConstants.endgameAngle)
                                .andThen(new WaitCommand(.75))
                                .deadlineFor(new SequentialCommandGroup(
                                                // new WaitCommand(.5),
                                                new SftCmd(m_sft,
                                                                SftConstants.endgameSetPoint))
                                                .alongWith(new InstantCommand(() -> System.out.println("sftOPENRan"))));
                Command sftDump = new CoralElevatorIntegratedCmd(m_coral, m_elevator, 0,
                                CoralIntakeConstants.endgameAngle)
                                .andThen(new WaitCommand(.75))
                                .deadlineFor(new SequentialCommandGroup(
                                                new SftCmd(m_sft,
                                                                SftConstants.dumpSetPoint))
                                                .alongWith(new InstantCommand(() -> System.out.println("sftDUMPRan"))));
                m_driveController.y()
                                .onTrue(new ConditionalCommand(sftOpen, sftClosedCmd, () -> getEndgamePoseState())
                                                .withTimeout(3.0))
                                .onFalse(new InstantCommand(() -> setEndgamePoseState(!endgameClosed)));
                m_driveController.x().onTrue(sftDump)
                                .onFalse(new InstantCommand(() -> setEndgamePoseState(false)));

        }

        public void configureSft() {
                SftAnalogCmd sftAnalogCmd = new SftAnalogCmd(m_sft,
                                () -> MathUtil.applyDeadband(m_copilotController.getRightY(), 0.1) * -.2);
                m_sft.setDefaultCommand(sftAnalogCmd);
                // m_controllerTwo.x().whileTrue(new SftCmd(m_sft,
                // SftConstants.endgameSetPoint))
                // .onFalse(new SftCmd(m_sft, 0));
        }

        private void configureAlerts() {
                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
                                .onTrue(
                                                controllerRumbleCommand()
                                                                .withTimeout(0.5)
                                                                .andThen(Commands.waitSeconds(4.75))
                                                                .repeatedly()
                                                                .withTimeout(15)

                                // .beforeStarting(() -> leds.endgameAlert = true)
                                // .finallyDo(() -> leds.endgameAlert = false)
                                );
                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
                                .onTrue(
                                                controllerRumbleCommand()
                                                                .withTimeout(0.1)
                                                                .andThen(Commands.waitSeconds(0.1))
                                                                .repeatedly()
                                                                .withTimeout(8)
                                // .beforeStarting(() -> leds.endgameAlert = true)
                                // .finallyDo(() -> leds.endgameAlert = false)
                                );
                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
                                .onTrue(
                                                controllerRumbleCommand()
                                                                .withTimeout(0.2)
                                                                .andThen(Commands.waitSeconds(0.3))
                                                                .repeatedly()
                                                                .withTimeout(10)
                                // .beforeStarting(() -> leds.endgameAlert = true)
                                // .finallyDo(() -> leds.endgameAlert = false)
                                );
                // Countdown
                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(endgameAlert3.get()))
                                .onTrue(
                                                controllerRumbleCommand()
                                                                .withTimeout(0.8)
                                                                .andThen(Commands.waitSeconds(0.2))
                                                                .repeatedly()
                                                                .withTimeout(5)
                                // .beforeStarting(() -> leds.endgameAlert = true)
                                // .finallyDo(() -> leds.endgameAlert = false)
                                );
                new Trigger(() -> (Math.abs(m_climber.getPosition() - ClimberConstants.upPosition2) < 0.005))
                                .onTrue(controllerRumbleCommand()
                                                .withTimeout(0.8)
                                                .andThen(Commands.waitSeconds(0.2))
                                                .repeatedly()
                                                .withTimeout(2));
        }

        public void configureAutoChooser() {
                // configureAutos();
                // System.out.println("!!!!!!!!!!!!!!!!finished of
                // configureAutos!!!!!!!!!!!!!!!!!!!");
                // Set up SysId routines
                autoChooser.addOption(
                                "Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(m_drive));
                autoChooser.addOption(
                                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption(
                                "Micah's test",
                                AutoBuilder.buildAuto("src\\main\\deploy\\pathplanner\\autos\\test.auto"));
        }

        public void configureClimber() {
                Command climberCmd = new ClimberAnalogCmd(m_climber,
                                () -> MathUtil.applyDeadband(m_copilotController.getLeftX(), 0.5));
                m_climber.setDefaultCommand(climberCmd);
                Command climberUpCmd = new ClimberUpCmd(m_climber);
                m_copilotController.y().whileTrue(climberUpCmd);

        }

        public void configureLeds() {
                // define commands
                Command ledAutnomousIndicator = new autonoumousIndicator(m_led);
                Command ledTeleopIndicator = new teleopStatesIndicators(m_led, m_coral);
                // create triggers
                Trigger autonomous = new Trigger(() -> DriverStation.isAutonomousEnabled());
                Trigger teleop = new Trigger(() -> DriverStation.isTeleopEnabled());
                // apply triggers
                autonomous.onTrue(ledAutnomousIndicator);
                teleop.onTrue(ledTeleopIndicator);
                // teleleop >done
                // autonomous >done
                // algea intak running
                // elevator level
                // coral intake >done
                // and placing running

        }

        public void configureCoralIntake() {

                Command coralIn = new CoralInCmd(m_coral, m_elevator);

                Command coralOut = new CoralOutCmd(m_coral);
                m_copilotController.a().onTrue(new InstantCommand(() -> m_coral.setSpeed(.15)));
                m_copilotController.a().onFalse(new InstantCommand(() -> m_coral.setSpeed(0)));
                m_copilotController.b().onTrue(new InstantCommand(() -> m_coral.setSpeed(-.15)));
                m_copilotController.b().onFalse(new InstantCommand(() -> m_coral.setSpeed(0)));
                // Command wristAnalog = new WristAnalogCmd(m_coral, () ->
                // m_copilotController.getRightX());
                Command slamCoral = new SlamCoralCmd(m_coral);
                // m_coral.setDefaultCommand(wristAnalog);
                // m_controllerTwo.y().whileTrue(slamCoral);
        }

        public void configureDrive() {
                // Default command, normal field-relative drive
                m_drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                m_drive,

                                                () -> -m_driveController.getLeftY(),
                                                () -> -m_driveController.getLeftX(),
                                                () -> -m_driveController.getRightX(),
                                                m_driveController.leftBumper(),
                                                () -> m_vision.getTx(),
                                                m_driveController.leftBumper(),
                                                m_driveController.rightBumper(),
                                                () -> !endgameClosed));

                // Lock to 0° when A button is held
                // m_driveController
                // .b()
                // .whileTrue(
                // DriveCommands.joystickDriveAtAngle(
                // m_drive,
                // () -> m_driveController.getLeftY(),
                // () -> m_driveController.getLeftX(),
                // () -> new Rotation2d()));

                // Switch to X pattern when X button is pressed
                m_driveController.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

                // Reset gyro to 0° when B button is pressed
                m_driveController
                                .a()
                                .onTrue(
                                                Commands.runOnce(
                                                                () -> m_drive.setPose(
                                                                                new Pose2d(m_drive.getPose()
                                                                                                .getTranslation(),
                                                                                                new Rotation2d())),
                                                                m_drive)
                                                                .ignoringDisable(true));
        }

        public void configureElevator() {

                Command homeElevator = new HomeElevatorCmd(m_elevator, m_coral);
                Command coralDefaultCommand = new CoralDefaultCmd(m_coral, m_elevator);
                Command elevatorAnalog = new ElevatorAnalogCmd(m_elevator, () -> m_controllerTwo.getLeftX());
                Command coralSmartDefualt = new ConditionalCommand(coralDefaultCommand, homeElevator,
                                () -> m_elevator.getLimitReset() && endgameClosed);
                // m_elevator.setDefaultCommand(elevatorAnalog);

                // m_elevator.setDefaultCommand(coralSmartDefualt);
                // m_coral.setDefaultCommand(coralSmartDefualt);

                m_copilotController.leftTrigger().whileTrue(coralSmartDefualt);

                m_controllerTwo.leftTrigger().whileTrue(new AlgaeRemovalCmd(m_drive, m_coral, m_elevator, () -> true));

                // new Trigger(() -> DriverStation.isTeleopEnabled() &&
                // !m_elevator.getLimitReset())
                // .whileTrue(homeElevator);

        }

        public void configureSetpoints() {

                Command homeElevator = new HomeElevatorCmd(m_elevator, m_coral);

                Command l1 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                new TunableNumber("Elevator/Height/L1", ElevatorConstants.coralLeveL1).get(),
                                new TunableNumber("Elevator/Angle/L1", CoralIntakeConstants.L1Angle).get());

                Command l2 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                new TunableNumber("Elevator/Height/L2", ElevatorConstants.coralLeveL2).get(),
                                new TunableNumber("Elevator/Angle/L2", CoralIntakeConstants.L2Angle).get());
                Command l3 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                new TunableNumber("Elevator/Height/L3", ElevatorConstants.coralLeveL3).get(),
                                new TunableNumber("Elevator/Angle/L3", CoralIntakeConstants.L3Angle).get());
                Command l4 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                new TunableNumber("Elevator/Height/L4", ElevatorConstants.coralLeveL4).get(),
                                new TunableNumber("Elevator/Angle/L4", CoralIntakeConstants.L4Angle).get());
                Command intake = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                                ElevatorConstants.intakePose, CoralIntakeConstants.IntakeAngle);
                Command smartSequentialCommand = new CoralSequentialCmd(m_drive, m_coral, m_elevator, true);
                // Command coralSource = new SourceParallelCmd(m_drive, m_coral, m_elevator);
                // m_controllerTwo.povUp().whileTrue(l4);
                // m_controllerTwo.povRight().whileTrue(l1);
                // m_controllerTwo.povDown().whileTrue(l2);
                // m_controllerTwo.povLeft().whileTrue(l3);
                // m_controllerTwo.leftBumper().whileTrue(intake);
                // m_driveController.rightBumper().onTrue(sequentialRight);
                // m_driveController.leftBumper().onTrue(sequentialLeft);
                // m_driveController.leftBumper().onFalse(new InstantCommand(() ->
                // cancelCommand(sequentialLeft)));
                // m_driveController.rightBumper().onFalse(new InstantCommand(() ->
                // cancelCommand(sequentialRight)));
                // m_driveController.leftBumper().whileTrue(leftDriveCommand);
                // m_driveController.rightBumper().whileTrue(rightDriveCommand);
                m_copilotController.povUp().onTrue(new InstantCommand(() -> CoralSequentialCmd.setLevel(4)));
                m_copilotController.povRight().onTrue(new InstantCommand(() -> CoralSequentialCmd.setLevel(1)));
                m_copilotController.povDown().onTrue(new InstantCommand(() -> CoralSequentialCmd.setLevel(2)));
                m_copilotController.povLeft().onTrue(new InstantCommand(() -> CoralSequentialCmd.setLevel(3)));
                m_copilotController.povUp().and(() -> override).whileTrue(l4);
                m_copilotController.povLeft().and(() -> override).whileTrue(l3);
                m_copilotController.povDown().and(() -> override).whileTrue(l2);
                m_copilotController.povRight().and(() -> override).whileTrue(l1);
                m_copilotController.leftBumper()
                                .onTrue(new InstantCommand(
                                                () -> CoralSequentialCmd.setAutonState(AutonState.LEFTREEF)));
                m_copilotController.leftBumper().and(() -> override)
                                .onTrue(new InstantCommand(() -> m_coral.setSpeed(0.5)));
                m_copilotController.leftBumper()
                                .onFalse(new InstantCommand(() -> m_coral.setSpeed(0)));
                m_copilotController.rightBumper().and(() -> override)
                                .onTrue(new InstantCommand(() -> m_coral.setSpeed(-0.5)));
                m_copilotController.rightBumper()
                                .onFalse(new InstantCommand(() -> m_coral.setSpeed(0)));
                m_copilotController.rightBumper()
                                .onTrue(new InstantCommand(
                                                () -> CoralSequentialCmd.setAutonState(AutonState.RIGHTREEF)));
                m_driveController.leftTrigger().whileTrue(smartSequentialCommand);

                // m_driveController.y().onTrue(coralSource);
                m_driveController.rightTrigger().whileTrue(
                                new DownToIntakeCmd(m_coral, m_elevator)
                                                .andThen(new UpFromIntakeCmd(m_coral, m_elevator)))
                                .onFalse(new UpFromIntakeCmd(m_coral, m_elevator));
                // m_driveController.leftBumper().whileTrue(new WristCmd(m_coral,
                // CoralIntakeConstants.IntakeAngle)
                // .withTimeout(1).andThen(new UpFromIntakeCmd(m_coral, m_elevator)));
        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public void configureAutos() {

                // Command homeElevator = new HomeElevatorCmd(m_elevator,m_coral);

                // Command l1 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                // new TunableNumber("Elevator/Height/L1", ElevatorConstants.coralLeveL1).get(),
                // new TunableNumber("Elevator/Angle/L1", CoralIntakeConstants.L1Angle).get());
                // Command l2 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                // new TunableNumber("Elevator/Height/L2", ElevatorConstants.coralLeveL2).get(),
                // new TunableNumber("Elevator/Angle/L2", CoralIntakeConstants.L2Angle).get());
                // Command l3 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                // new TunableNumber("Elevator/Height/L3", ElevatorConstants.coralLeveL3).get(),
                // new TunableNumber("Elevator/Angle/L3", CoralIntakeConstants.L3Angle).get());
                // Command l4 = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                // new TunableNumber("Elevator/Height/L4", ElevatorConstants.coralLeveL4).get(),
                // new TunableNumber("Elevator/Angle/L4", CoralIntakeConstants.L4Angle).get());
                // Command intakePos = new CoralElevatorIntegratedCmd(m_coral, m_elevator,
                // ElevatorConstants.intakePose, CoralIntakeConstants.IntakeAngle);
                // Command coralIn = new CoralInCmd(m_coral);
                // Command slamCoral = new CoralCmd(m_coral, .05, -.2);
                // NamedCommands.registerCommand("homeElevator", homeElevator);
                // NamedCommands.registerCommand("l4", l4);
                // NamedCommands.registerCommand("l3", l3);
                // NamedCommands.registerCommand("l2", l2);
                // NamedCommands.registerCommand("l1", l1);
                // NamedCommands.registerCommand("intakePos", intakePos);
                // NamedCommands.registerCommand("coralIn", coralIn);
                // NamedCommands.registerCommand("slamCoral", slamCoral);
        }

        public void toggleOverride() {
                override = !override;
                Logger.recordOutput("Override", override);
        }

        private void setEndgamePoseState(boolean state) {

                endgameClosed = state;
                Logger.recordOutput("SFT/endgameClosed", endgameClosed);
        }

        @AutoLogOutput(key = "SFT/getEndgameClosed")
        private boolean getEndgamePoseState() {
                return endgameClosed;
        }

        // Creates controller rumble command
        private Command controllerRumbleCommand() {
                return Commands.startEnd(
                                () -> {
                                        m_driveController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                                        m_copilotController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                                },
                                () -> {
                                        m_driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                        m_copilotController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                });
        }

}
