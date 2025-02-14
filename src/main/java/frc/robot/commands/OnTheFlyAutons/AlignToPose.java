/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */

package frc.robot.commands.OnTheFlyAutons;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

/**
 * A {@link Command} that autonomous drives and aligns the robot to the
 * specified branch of the
 * nearest reef side.
 */
public class AlignToPose extends Command {
  private static final DataLog LOG = DataLogManager.getLog();
  private static final double MAX_TRANSLATIONAL_POWER = 0.30;
  private static final double MAX_ROTATIONAL_POWER = 0.5;

  protected final Drive drivetrain;

  protected Pose2d targetPose;

  private final PIDController xController = new PIDController(AutonConstants.driveKp, 0, 0);
  private final PIDController yController = new PIDController(AutonConstants.driveKp, 0, 0);
  private final PIDController rController = new PIDController(AutonConstants.turnKp, 0, 0);

  private final DoubleLogEntry xErrorLog = new DoubleLogEntry(LOG, "Vision/Pose X Error");
  private final DoubleLogEntry yErrorLog = new DoubleLogEntry(LOG, "Vision/Pose Y Error");
  private final DoubleLogEntry rErrorLog = new DoubleLogEntry(LOG, "Vision/Pose Yaw Error");

  private final DoubleLogEntry xTargetLog = new DoubleLogEntry(LOG, "Vision/Pose Target X");
  private final DoubleLogEntry yTargetLog = new DoubleLogEntry(LOG, "Vision/Pose Target Y");
  private final DoubleLogEntry rTargetLog = new DoubleLogEntry(LOG, "Vision/Pose Target Yaw");

  private double xTarget;
  private double yTarget;
  private double rTarget; // in degrees

  /** Creates a new {@link AlignToPose} command. */
  public AlignToPose(Drive drive, Pose2d targetPose) {
    this.drivetrain = drive;
    this.targetPose = targetPose;
    // Declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Target pose: " + targetPose);

    xTarget = targetPose.getX();
    yTarget = targetPose.getY();
    rTarget = targetPose.getRotation().getDegrees();

    xTargetLog.append(xTarget);
    yTargetLog.append(yTarget);
    rTargetLog.append(rTarget);

    // Set up the PID controllers to drive to the target pose.
    xController.setSetpoint(xTarget);
    yController.setSetpoint(yTarget);
    rController.setSetpoint(rTarget);

    xController.setPID(AutonConstants.driveKp, 0, 0);
    yController.setPID(AutonConstants.driveKp, 0, 0);
    rController.setPID(AutonConstants.turnKp, 0, 0);

    xController.setTolerance(AutonConstants.maxLinearError);
    yController.setTolerance(AutonConstants.maxLinearError);
    rController.setTolerance(AutonConstants.maxRotationalError);

    rController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // run pid
    Pose2d currentRobotPose = drivetrain.getPose();
    double currentX = currentRobotPose.getX();
    double currentY = currentRobotPose.getY();
    double currentR = currentRobotPose.getRotation().getDegrees();

    xErrorLog.append(xTarget - currentX);
    yErrorLog.append(yTarget - currentY);
    rErrorLog.append(rTarget - currentR);

    double xSpeed = MathUtil.clamp(
        xController.calculate(currentX), -MAX_TRANSLATIONAL_POWER, MAX_TRANSLATIONAL_POWER);
    double ySpeed = MathUtil.clamp(
        yController.calculate(currentY), -MAX_TRANSLATIONAL_POWER, MAX_TRANSLATIONAL_POWER);
    double rSpeed = MathUtil.clamp(
        rController.calculate(currentR), -MAX_ROTATIONAL_POWER, MAX_ROTATIONAL_POWER);
    Logger.recordOutput("Autons/xVelocity", xSpeed);
    Logger.recordOutput("Autons/yVelocity", ySpeed);
    Logger.recordOutput("Autons/RotationalVelocity", rSpeed);
    Logger.recordOutput("Autons/Trajectory", currentRobotPose, targetPose);
    drivetrain.runVelocity(new ChassisSpeeds(xSpeed, ySpeed, Units.degreesToRadians(rSpeed)));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // drivetrain.stopMotors();
    drivetrain.runVelocity(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint();
  }
}
