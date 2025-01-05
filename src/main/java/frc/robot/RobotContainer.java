// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.MotorMMove;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.MotorMoveCmd;
import frc.robot.Logging;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final MotorMMove m_motorMMove = new MotorMMove();
  // Replace with CommandPS4Controller or CommandJoystick if needed
     private final CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureSwerve();
    configureMotor();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureSwerve() {
    SwerveJoystickCmd swerveJoystickCmd = new SwerveJoystickCmd(m_swerveSubsystem,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
            OIConstants.kDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
            OIConstants.kDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(),
            OIConstants.kDeadband));
    m_swerveSubsystem.setDefaultCommand(swerveJoystickCmd);
    m_driverController.a().onTrue(new InstantCommand(() -> m_swerveSubsystem.setFieldRelative()));
    m_driverController.b().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroHeading()));
    // m_driverController.x().onTrue(new InstantCommand(() ->
    // AlignCommands.alignToAmp(m_visionPoseEstimator).schedule()));

  }
  private void configureMotor(){
    MotorMoveCmd motorMoveCmd = new MotorMoveCmd(m_motorMMove);
    m_driverController.leftTrigger().onTrue(motorMoveCmd);
  }
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
