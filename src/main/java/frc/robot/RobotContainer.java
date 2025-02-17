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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeAnalogCmd;
import frc.robot.commands.AlgaeCmd;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.goToCommands.AutonConstants.PoseConstants;
import frc.robot.commands.algaeCommands.AlgaeDefaultCmd;
import frc.robot.commands.algaeCommands.AlgaeDownCmd;
import frc.robot.commands.algaeCommands.AlgaeShootCmd;
import frc.robot.commands.autonCommands.CoralSequentialCmd;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.TunableNumber;
import frc.robot.util.TunableNumber;
import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.algae.AlgaeIOSparkMax;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.coralIntake.CoralIntake;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.subsystems.coralIntake.CoralIntakeIO;
import frc.robot.subsystems.coralIntake.CoralIntakeIOSparkMax;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.commands.coralCommands.CoralCmd;
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
import frc.robot.commands.coralCommands.ElevatorCmd;
import frc.robot.commands.coralCommands.HomeElevatorCmd;
import frc.robot.commands.coralCommands.ElevatorAnalogCmd;
import frc.robot.commands.coralCommands.WristCmd;
import frc.robot.commands.coralCommands.WristAnalogCmd;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import java.util.List;

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
  private ClimberSubsystem m_climber;
  private AlgaeSubsystem m_algae;
  // Controller
  private final CommandXboxController m_driveController = new CommandXboxController(0);
  private final CommandXboxController m_copilotController = new CommandXboxController(1);
  private final CommandXboxController m_controllerTwo = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_climber = new ClimberSubsystem(new ClimberIOSparkMax());
        m_algae = new AlgaeSubsystem(new AlgaeIOSparkMax());
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
    configureAlgae();
    configureClimber();
    configureCoralIntake();
    configureDrive();
    configureElevator();
    configureSetpoints();
    configureAutos();
  }

  public void cancelCommand(Command cmd) {
    if (cmd.isScheduled()) {

      System.out.println(cmd + " canceled");

      cmd.cancel();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void configureAlgae() {
    Command algaeDefaultCmd = new AlgaeDefaultCmd(m_algae);
    Command algaeIntakeCmd = new AlgaeDownCmd(m_algae);
    Command algaeShootCmd = new AlgaeShootCmd(m_algae);
    m_copilotController.a().whileTrue(algaeIntakeCmd);
    m_copilotController.b().whileTrue(algaeShootCmd);
    // m_algae.setDefaultCommand(algaeDefaultCmd);
  }

  public void configureAutoChooser() {
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
    Command climberCmd = new ClimberAnalogCmd(m_climber, () -> m_copilotController.getRightX());
    m_climber.setDefaultCommand(climberCmd);
    Command climberUpCmd = new ClimberUpCmd(m_climber);
    m_driveController.povUp().whileTrue(climberUpCmd);

  }

  public void configureCoralIntake() {
    Command coralIn = new CoralInCmd(m_coral);
    Command coralOut = new CoralOutCmd(m_coral);
    Command wrist = new WristCmd(m_coral, new TunableNumber("WristAngle", CoralIntakeConstants.anglevalue).get());
    Command wristAnalog = new WristAnalogCmd(m_coral, () -> m_controllerTwo.getRightX());
    Command slamCoral = new CoralCmd(m_coral, .05, -.2);
    // m_coral.setDefaultCommand(wristAnalog);
    // m_controllerTwo.povUp().onTrue(l1);
    // m_controllerTwo.povRight().onTrue(l2);
    // m_controllerTwo.povDown().onTrue(l3);
    // m_controllerTwo.povLeft().onTrue(l4);
    m_coral.setDefaultCommand(wristAnalog);
    m_driveController.a().onTrue(new InstantCommand(() -> m_coral.setSpeed(.15)));
    m_driveController.a().onFalse(new InstantCommand(() -> m_coral.setSpeed(0)));
    m_driveController.b().onTrue(new InstantCommand(() -> m_coral.setSpeed(-.15)));
    m_driveController.b().onFalse(new InstantCommand(() -> m_coral.setSpeed(0)));
    // m_controllerTwo.b().whileTrue(coralOut); // change back to copilot after
    // testing// Subject to Change
    m_controllerTwo.y().whileTrue(slamCoral);
  }

  public void configureDrive() {
    // Default command, normal field-relative drive
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> m_driveController.getLeftY(),
            () -> m_driveController.getLeftX(),
            () -> -m_driveController.getRightX()));

    // Lock to 0° when A button is held
    m_driveController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_drive,
                () -> m_driveController.getLeftY(),
                () -> m_driveController.getLeftX(),
                () -> new Rotation2d()));

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
    Command alignLeftReef = new DriveToPose(m_drive, () -> PoseConstants.leftReef);
    m_driveController.leftBumper().whileTrue(alignLeftReef);
    Command alignRightReef = new DriveToPose(m_drive, () -> PoseConstants.rightReef);
    m_driveController.rightBumper().whileTrue(alignRightReef);
    Command alignCoralStation = new DriveToPose(m_drive, () -> PoseConstants.rightReef);
    m_driveController.y().whileTrue(alignCoralStation);
    Command goToNearestRightCommand = new DriveToNearest(m_drive, () -> PoseConstants.rightReefPoints());
    m_controllerTwo.rightTrigger().whileTrue(goToNearestRightCommand);
  }

  public void configureElevator() {

    Command elevatorAnalog = new ElevatorAnalogCmd(m_elevator, () -> m_controllerTwo.getLeftX());
    m_elevator.setDefaultCommand(elevatorAnalog);
  }

  public void configureSetpoints() {
    Command homeElevator = new HomeElevatorCmd(m_elevator);
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
    Command sequential = new CoralSequentialCmd(m_drive, m_coral, m_elevator, false, 3, true);
    m_controllerTwo.povUp().whileTrue(l1);
    m_controllerTwo.povRight().whileTrue(l2);
    m_controllerTwo.povDown().whileTrue(l3);
    m_controllerTwo.povLeft().whileTrue(l4);
    m_controllerTwo.leftBumper().whileTrue(intake);
    m_driveController.leftTrigger().whileTrue(sequential);
    m_driveController.rightTrigger().whileTrue(homeElevator);

    // The Below command is ONLY for testing and should be removed in the final
    // build. This allows you to zero the elevator without a limit switch
    // m_controllerTwo.leftBumper().onTrue(new InstantCommand(() ->
    // m_elevator.zeroElevator()));
    m_controllerTwo.leftTrigger().whileTrue(homeElevator);
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void configureAutos() {
    Command homeElevator = new HomeElevatorCmd(m_elevator);
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
    Command coralIn = new CoralInCmd(m_coral);
    Command slamCoral = new CoralCmd(m_coral, .05, -.2);
    NamedCommands.registerCommand("homeElevator", homeElevator);
    NamedCommands.registerCommand("l4", l4);
    NamedCommands.registerCommand("l3", l3);
    NamedCommands.registerCommand("l2", l2);
    NamedCommands.registerCommand("l1", l1);
    NamedCommands.registerCommand("intakePos", intakePos);
    NamedCommands.registerCommand("coralIn", coralIn);
    NamedCommands.registerCommand("slamCoral", slamCoral);
  }
}
