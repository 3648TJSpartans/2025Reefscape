package frc.robot.commands;

import com.pathplanner.lib.config.PIDConstants;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.Geometry2D;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class SwerveAutoAlignStraight extends Command {
  private final Pose2d redPose;
  private final Pose2d bluePose;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController rotationController;

  private Pose2d targetPose;
  private final Timer timer = new Timer();
  private final Drive drive;

  public SwerveAutoAlignStraight(Pose2d redPose, Pose2d bluePose, Drive drive) {
    this.drive = drive;
    this.redPose = redPose;
    this.bluePose = bluePose;

    this.xController = new ProfiledPIDController(
        DriveConstants.driveKp,
        0,
        DriveConstants.driveKd,
        new TrapezoidProfile.Constraints(3.0, 2.0));
    this.yController = new ProfiledPIDController(
        DriveConstants.driveKp,
        0,
        DriveConstants.driveKd,
        new TrapezoidProfile.Constraints(3.0, 2.0));
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
    if (Robot.isRedAlliance()) {
      targetPose = redPose;
    } else {
      targetPose = bluePose;
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
    double xDif = Math.abs(currentPose.getX() - targetPose.getX());
    double yDif = Math.abs(currentPose.getY() - targetPose.getY());
    if (xDif < 0.025) {
      xVel = 0;
    }
    if (yDif < 0.025) {
      yVel = 0;
    }
    if (Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 15) {
      rotVel = 0;
    }
    double normalXDif = Math.max(xDif / Math.sqrt(xDif * xDif + yDif * yDif), .5);
    double normalYDif = Math.max(yDif / Math.sqrt(xDif * xDif + yDif * yDif), .5);
    Logger.recordOutput("Autons/Trajectory", currentPose, targetPose);
    Logger.recordOutput("Autons/xVelocity", xVel);
    Logger.recordOutput("Autons/yVelocity", yVel);
    Logger.recordOutput("Autons/RotationalVelocity", rotVel);
    drive.runVelocity(new ChassisSpeeds(xVel * normalXDif, yVel * normalYDif, rotVel));
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drive.getPose();
    return Math.abs(currentPose.getX() - targetPose.getX()) < 0.025
        && Math.abs(currentPose.getY() - targetPose.getY()) < 0.025
        && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 15;
  }
}
