package frc.robot.commands.OnTheFlyAutons;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;

public class AutonConstants {
    public static final double maxVelocity = 4.0;
    public static final double maxAcceleration = 2.0;
    public static final double maxLinearVelocity = maxVelocity / 1.412; // Max velocity in x or y direction going at 45
                                                                        // degree
    // angle
    public static final double driveKp = DriveConstants.driveKp;
    public static final double driveKd = DriveConstants.driveKd;
    public static final double turnKp = DriveConstants.turnKp;
    public static final double turnKd = DriveConstants.turnKd;

    public static final double maxError = 0.035; // Meters
    public static final double maxLinearError = maxError / 1.412;
    public static final double maxRotationalError = 10; // degrees

    public static class PoseConstants {
        public final static Pose2d rightReef = new Pose2d(3, 3.76, Rotation2d.fromDegrees(0));
        public final static Pose2d leftReef = new Pose2d(3, 4.23, Rotation2d.fromDegrees(0));
        public final static Pose2d coralStation = new Pose2d(1.5, 1.6, Rotation2d.fromDegrees(0));
        public final static Pose2d[] criticalPoints = new Pose2d[] {
                new Pose2d(2.423, 3, Rotation2d.fromDegrees(0)),
                new Pose2d(2.423, 5, Rotation2d.fromDegrees(0))
        };
    }
}
