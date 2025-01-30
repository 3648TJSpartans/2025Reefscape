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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SwerveAutoAlignPose;
import frc.robot.commands.SwerveAutoAlignPoseNearest;
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
    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations

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
        Pose2d rightReef = new Pose2d(3, 3.76, Rotation2d.fromDegrees(0));
        Pose2d leftReef = new Pose2d(3, 4.23, Rotation2d.fromDegrees(0));
        Pose2d coralStation = new Pose2d(1.5, 1.6, Rotation2d.fromDegrees(0));
        Command alignLeftReef = new SwerveAutoAlignPose(leftReef, leftReef, drive);
        controller.leftBumper().onTrue(alignLeftReef);
        controller.leftBumper().onFalse(new InstantCommand(() -> cancelCommand(alignLeftReef)));
        Command alignRightReef = new SwerveAutoAlignPose(rightReef, rightReef, drive);
        controller.rightBumper().onTrue(alignRightReef);
        controller.rightBumper().onFalse(new InstantCommand(() -> cancelCommand(alignRightReef)));
        Command alignCoralStation = new SwerveAutoAlignPose(coralStation, coralStation, drive);
        controller.y().onTrue(alignCoralStation);
        controller.y().onFalse(new InstantCommand(() -> cancelCommand(alignCoralStation)));
        Command goToNearestCommand = new SwerveAutoAlignPoseNearest(drive);
        controller.rightTrigger().onTrue(goToNearestCommand);
        controller.rightTrigger().onFalse(new InstantCommand(() -> cancelCommand(goToNearestCommand)));
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
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void configureAutons() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }
}
