package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class SwerveAutoAlignPoseNearest extends Command {

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController rotationController;

  private Pose2d targetPose;
  private final Timer timer = new Timer();
  private final Drive drive;
  private final Pose2d[] points = new Pose2d[] {
      new Pose2d(2.423, 3, Rotation2d.fromDegrees(0)),
      new Pose2d(2.423, 5, Rotation2d.fromDegrees(0))
  };

  public SwerveAutoAlignPoseNearest(Drive drive) {
    this.drive = drive;

    this.xController = new ProfiledPIDController(
        DriveConstants.driveKp,
        0,
        DriveConstants.driveKd,
        new TrapezoidProfile.Constraints(2.0, 2.0));
    this.yController = new ProfiledPIDController(
        DriveConstants.driveKp,
        0,
        DriveConstants.driveKd,
        new TrapezoidProfile.Constraints(2.0, 2.0));
    new TrapezoidProfile.Constraints(2.0, 2.0);
    this.rotationController = new ProfiledPIDController(
        DriveConstants.driveKp,
        0,
        DriveConstants.driveKd,
        new TrapezoidProfile.Constraints(2.0, 2.0));
    new TrapezoidProfile.Constraints(
        Units.degreesToRadians(180), Units.degreesToRadians(180));
    this.rotationController.setIZone((new PIDConstants(6, 0, 0, Units.degreesToRadians(20)).iZone));
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    targetPose = points[0];
    double minDistance = Double.MAX_VALUE;
    for (Pose2d point : points) {
      double distance = drive.getPose().getTranslation().getDistance(point.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        targetPose = point;
      }
    }

    Pose2d currentPose = drive.getPose();
    ChassisSpeeds currentSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
        drive.getChassisSpeeds(), currentPose.getRotation());

    xController.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
    yController.reset(currentPose.getY(), currentSpeeds.vyMetersPerSecond);
    rotationController.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();

    double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotFeedback = rotationController.calculate(
        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    double xFF = xController.getSetpoint().velocity;
    double yFF = yController.getSetpoint().velocity;
    double rotFF = rotationController.getSetpoint().velocity;

    double xVel = xFF + xFeedback;
    double yVel = yFF + yFeedback;
    double rotVel = rotFF + rotFeedback;

    if (Math.abs(currentPose.getX() - targetPose.getX()) < 0.025) {
      xVel = 0;
    }
    if (Math.abs(currentPose.getY() - targetPose.getY()) < 0.025) {
      yVel = 0;
    }
    if (Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 15) {
      rotVel = 0;
    }

    drive.runVelocity(new ChassisSpeeds(xVel, yVel, rotVel));
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drive.getPose();
    return Math.abs(currentPose.getX() - targetPose.getX()) < 0.025
        && Math.abs(currentPose.getY() - targetPose.getY()) < 0.025
        && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 15;
  }
}
