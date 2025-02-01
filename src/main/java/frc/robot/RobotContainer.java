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
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.PoseEstimator;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AlgaeCmd;
import frc.robot.commands.DriveCommands;

import frc.robot.subsystems.Algae.AlgaeIOSparkMax;
import frc.robot.subsystems.Algae.AlgaeSubsystem;

import frc.robot.commands.SwerveAutoAlignStraight;
import frc.robot.commands.OnTheFlyAutons.AutonConstants.PoseConstants;
import frc.robot.commands.OnTheFlyAutons.SwerveAutoAlignPose;
import frc.robot.commands.OnTheFlyAutons.SwerveAutoAlignPoseNearest;
import frc.robot.commands.AlignCommands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;

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
  private final Drive drive;
  private final Vision vision;
  private AlgaeSubsystem algaeSubsystem;
  private AlgaeCmd algaeCmd;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController copilotController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        algaeSubsystem = new AlgaeSubsystem(new AlgaeIOSparkMax());
        drive = new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3));
        vision = new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight("limelight-three", drive::getRotation),
            new VisionIOLimelight("limelight-twoplus",
                drive::getRotation));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        vision = new Vision(
            drive::addVisionMeasurement,
            // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0,
            // drive::getPose),
            new VisionIOPhotonVisionSim(VisionConstants.camera1Name,
                VisionConstants.robotToCamera1,
                drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
        }, new VisionIO() {
        });
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Micah's test",
        AutoBuilder.buildAuto("src\\main\\deploy\\pathplanner\\autos\\test.auto"));
    // Configure the button bindings

    // Configure the button bindings
    configureButtonBindings();
    configureAlgae();
    // configureAutons();
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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(
                    new Pose2d(drive.getPose()
                        .getTranslation(),
                        new Rotation2d())),
                drive)
                .ignoringDisable(true));
    // controller.x().onTrue(AlignCommands.goTo(drive));
    // controller.leftTrigger().whileTrue(m_AlignCommands.goTo(drive));

    Command alignLeftReef = new SwerveAutoAlignPose(PoseConstants.leftReef, PoseConstants.leftReef, drive);
    controller.leftBumper().whileTrue(alignLeftReef);
    Command alignRightReef = new SwerveAutoAlignPose(PoseConstants.rightReef, PoseConstants.rightReef, drive);
    controller.rightBumper().whileTrue(alignRightReef);
    Command alignCoralStation = new SwerveAutoAlignPose(PoseConstants.coralStation, PoseConstants.coralStation,
        drive);
    controller.y().whileTrue(alignCoralStation);
    Command goToNearestCommand = new SwerveAutoAlignPoseNearest(drive);
    controller.rightTrigger().whileTrue(goToNearestCommand);
  }

  public void cancelCommand(Command cmd) {
    if (cmd.isScheduled()) {
      cmd.cancel();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public void configureAlgae() {
    // algaeCmd = new AlgaeCmd(algaeSubsystem);
    controller.rightTrigger().onTrue(new InstantCommand(() -> algaeSubsystem.setLiftPosition(0)));// change the zero
    controller.rightBumper().onTrue(new InstantCommand(() -> algaeSubsystem.setIntakeSpeed(0.5)));
    controller.rightBumper().onFalse(new InstantCommand(() -> algaeSubsystem.setIntakeSpeed(0)));
    controller.leftBumper().onTrue(new InstantCommand(() -> algaeSubsystem.setIntakeSpeed(-0.5)));
    controller.leftBumper().onFalse(new InstantCommand(() -> algaeSubsystem.setIntakeSpeed(0)));
    algaeSubsystem.setDefaultCommand(algaeCmd);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void configureAutons() {
    controller.leftTrigger().whileTrue(Commands.runOnce(() -> {
      Pose2d currentPose = drive.getPose();
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(1.0, 0.0)),
          new Rotation2d());

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          new PathConstraints(
              4.0, 4.0,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          null, // Ideal starting state can be null for on-the-fly paths
          new GoalEndState(1, currentPose.getRotation()));

      // Prevent this path from being flipped on the red alliance, since the given
      // positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));

  }

  public Command goToPoint(Pose2d targetPose) {
    return Commands.runOnce(() -> {
      Pose2d currentPose = drive.getPose();

      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = targetPose;

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          waypoints,
          new PathConstraints(
              4.0, 4.0,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          null, // Ideal starting state can be null for on-the-fly paths
          new GoalEndState(1, currentPose.getRotation()));

      // Prevent this path from being flipped on the red alliance, since the given
      // positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    });
  }
}
