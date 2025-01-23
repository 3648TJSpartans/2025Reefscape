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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
<<<<<<< Updated upstream
import frc.robot.subsystems.vision.VisionIOPhotonVision;
=======
import frc.robot.subsystems.vision.VisionIOLimelight;
>>>>>>> Stashed changes
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

        // Controller
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);
        private final CommandXboxController m_copilotController = new CommandXboxController(
                        OIConstants.kCopilotControllerPort);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations

<<<<<<< Updated upstream
                                m_drive = new Drive(
                                                new GyroIONavX(),
                                                new ModuleIOSpark(0),
                                                new ModuleIOSpark(1),
                                                new ModuleIOSpark(2),
                                                new ModuleIOSpark(3));
                                m_vision = new Vision(
                                                m_drive::addVisionMeasurement,
                                                // new VisionIOPhotonVision(camera0Name, m_drive::getRotation),
                                                new VisionIOPhotonVision(VisionConstants.camera1Name,
                                                                VisionConstants.robotToCamera1));
                                break;
=======
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-three", drive::getRotation),
                new VisionIOLimelight("limelight-twoplus", drive::getRotation));
        break;
>>>>>>> Stashed changes

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
                                break;
                }

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
                // Default command, normal field-relative m_drive
                m_drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                m_drive,
                                                () -> m_driverController.getLeftY(),
                                                () -> m_driverController.getLeftX(),
                                                () -> m_driverController.getRightX()));

                // Lock to 0° when A button is held
                m_driverController
                                .b()
                                .whileTrue(
                                                DriveCommands.joystickDriveAtAngle(
                                                                m_drive,
                                                                () -> m_driverController.getLeftY(),
                                                                () -> m_driverController.getLeftX(),
                                                                () -> new Rotation2d()));

                // Switch to X pattern when X button is pressed
                m_driverController.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

                // Reset gyro to 0° when B button is pressed
                m_driverController
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

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}
