package frc.robot.commands.OnTheFlyAutons;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveConstants;

public class AutonConstants {
    public static final double maxVelocity = 3.0;
    public static final double maxAngularVelocity = 3.0;
    public static final double maxAngularAcceleration = 3.0;
    public static final double maxAcceleration = 2.0;
    public static final double maxLinearVelocity = maxVelocity / 1.412; // Max velocity in x or y direction going at 45
                                                                        // degree
    // angle
    public static final double driveKp = DriveConstants.driveKp;
    public static final double driveKd = DriveConstants.driveKd;
    public static final double turnKp = DriveConstants.turnKp;
    public static final double turnKd = DriveConstants.turnKd;

    public static final double maxError = 0.025; // Meters
    public static final double maxLinearError = maxError / 1.412;
    public static final double maxRotationalError = 5; // degrees

    public static class PoseConstants {
        public final static Pose2d rightReef = new Pose2d(2.9, 3.73, Rotation2d.fromDegrees(0));
        public final static Pose2d leftReef = new Pose2d(2.9, 4.17, Rotation2d.fromDegrees(0));
        public final static Pose2d coralStation = new Pose2d(1.5, 1.6, Rotation2d.fromDegrees(0));
        public final static Pose2d[] blueRightReefPoints = new Pose2d[] {
                new Pose2d(2.95, 3.85, Rotation2d.fromDegrees(0)), // Pose A
                new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60)), // Pose C
                new Pose2d(5.44, 2.75, Rotation2d.fromDegrees(120)), // Pose E
                new Pose2d(6.08, 4.12, Rotation2d.fromDegrees(180)), // Pose G
                new Pose2d(5.13, 5.51, Rotation2d.fromDegrees(240)), // Pose I
                new Pose2d(3.54, 5.33, Rotation2d.fromDegrees(300)) // Pose K
        };
        public final static Pose2d[] blueLeftReefPoints = new Pose2d[] {
                new Pose2d(2.95, 4.12, Rotation2d.fromDegrees(0)), // Pose B
                new Pose2d(3.8, 2.59, Rotation2d.fromDegrees(60)), // Pose D
                new Pose2d(5.17, 2.61, Rotation2d.fromDegrees(120)), // Pose F
                new Pose2d(6.08, 3.78, Rotation2d.fromDegrees(180)), // Pose H
                new Pose2d(5.41, 5.35, Rotation2d.fromDegrees(240)), // Pose J
                new Pose2d(3.95, 5.53, Rotation2d.fromDegrees(300)) // Pose L
        };
        public final static Pose2d[] redRightReefPoints = new Pose2d[] {
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose A
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose C
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose E
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose G
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose I
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)) // Pose K
        };
        public final static Pose2d[] redLeftReefPoints = new Pose2d[] {
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose B
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose D
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose F
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose H
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // Pose J
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)) // Pose L
        };

        public static Pose2d[] rightReefPoints() {
            if (Robot.isRedAlliance()) {
                return redRightReefPoints;
            } else {
                return blueRightReefPoints;
            }
        }

        public static Pose2d[] leftReefPoints() {
            if (Robot.isRedAlliance()) {
                return redLeftReefPoints;
            } else {
                return blueLeftReefPoints;
            }
        }
    }
}
