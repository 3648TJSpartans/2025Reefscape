package frc.robot.commands.OnTheFlyAutons;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

public class AutonConstants {
    public static final double drivekP = 1.0;
    public static final double drivekD = 0.0;
    public static final double thetakP = 5.0;
    public static final double thetakD = 0.0;
    public static final double driveMaxVelocity = Units.inchesToMeters(150.0 / 4);
    public static final double driveMaxAcceleration = Units.inchesToMeters(95.0 / 4);
    public static final double thetaMaxVelocity = Units.degreesToRadians(360.0 / 2);
    public static final double thetaMaxAcceleration = Units.degreesToRadians(720.0 / 2);
    public static final double driveTolerance = 0.03;
    public static final double thetaTolerance = Units.degreesToRadians(2.5);
    public static final double ffMinRadius = 0.2;
    public static final double ffMaxRadius = 0.6;

    public static class PoseConstants {
        public final static Pose2d rightReef = new Pose2d(2.9, 3.73, Rotation2d.fromDegrees(0));
        public final static Pose2d leftReef = new Pose2d(2.9, 4.17, Rotation2d.fromDegrees(0));
        public final static Pose2d coralStation = new Pose2d(1.5, 1.6, Rotation2d.fromDegrees(0));
        public final static Pose2d[] criticalPoints = new Pose2d[] {
                new Pose2d(2.423, 3, Rotation2d.fromDegrees(0)),
                new Pose2d(2.423, 5, Rotation2d.fromDegrees(0))
        };
    }
}
