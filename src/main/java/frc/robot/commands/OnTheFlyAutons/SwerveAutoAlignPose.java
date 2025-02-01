package frc.robot.commands.OnTheFlyAutons;

import com.pathplanner.lib.config.PIDConstants;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;

public class SwerveAutoAlignPose extends Command {
  private final Pose2d redPose;
  private final Pose2d bluePose;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController rotationController;

  private Pose2d targetPose;
  private final Drive drive;

  public SwerveAutoAlignPose(Pose2d redPose, Pose2d bluePose, Drive drive) {
    this.drive = drive;
    this.redPose = redPose;
    this.bluePose = bluePose;

    this.xController = new ProfiledPIDController(
        AutonConstants.driveKp,
        0,
        AutonConstants.driveKd,
        new TrapezoidProfile.Constraints(AutonConstants.maxLinearVelocity, AutonConstants.maxAcceleration));
    this.yController = new ProfiledPIDController(
        AutonConstants.driveKp,
        0,
        AutonConstants.driveKd,
        new TrapezoidProfile.Constraints(AutonConstants.maxLinearVelocity, AutonConstants.maxAcceleration));
    new TrapezoidProfile.Constraints(AutonConstants.maxLinearVelocity, AutonConstants.maxAcceleration);
    this.rotationController = new ProfiledPIDController(
        AutonConstants.turnKp,
        0,
        AutonConstants.turnKd,
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
    if (Math.abs(currentPose.getX() - targetPose.getX()) < AutonConstants.maxLinearError) {
      xVel = 0;
    }
    if (Math.abs(currentPose.getY() - targetPose.getY()) < AutonConstants.maxLinearError) {
      yVel = 0;
    }
    if (Math.abs(
        currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < AutonConstants.maxRotationalError) {
      rotVel = 0;
    }
    Logger.recordOutput("Autons/Trajectory", currentPose, targetPose);
    Logger.recordOutput("Autons/xVelocity", xVel);
    Logger.recordOutput("Autons/yVelocity", yVel);
    Logger.recordOutput("Autons/RotationalVelocity", rotVel);
    drive.runVelocity(new ChassisSpeeds(xVel, yVel, rotVel));
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = drive.getPose();
    return Math.abs(currentPose.getX() - targetPose.getX()) < AutonConstants.maxLinearError
        && Math.abs(currentPose.getY() - targetPose.getY()) < AutonConstants.maxLinearError
        && Math.abs(
            currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < AutonConstants.maxRotationalError;
  }
}
