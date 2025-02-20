package frc.robot.commands.goToCommands;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.AllianceFlipUtil;

public class AutonConstants {
    public static final double drivekP = 1.0;
    public static final double drivekD = 0.0;
    public static final double thetakP = 5.0;
    public static final double thetakD = 0.0;
    public static final double driveMaxVelocity = Units.inchesToMeters(150.0 / 4);
    public static final double driveMaxAcceleration = Units.inchesToMeters(95.0 / 4);
    public static final double thetaMaxVelocity = Units.degreesToRadians(360.0 / 2);
    public static final double thetaMaxAcceleration = Units.degreesToRadians(720.0 / 2);
    public static final double driveTolerance = 0.1; // 0.03
    public static final double thetaTolerance = Units.degreesToRadians(2.5); // 2.5
    public static final double ffMinRadius = 0.2;
    public static final double ffMaxRadius = 0.6;
    public static final int defaultLevel = 4;

    public static class PoseConstants {
        public final static Pose2d rightReef = new Pose2d(2.9, 3.73, Rotation2d.fromDegrees(0));
        public final static Pose2d leftReef = new Pose2d(2.9, 4.17, Rotation2d.fromDegrees(0));
        public final static double fieldLength = 17.548;
        public final static double fieldWidth = 8.042;

        private static Pose2d[] blueCoralStations = new Pose2d[] {
                new Pose2d(1.127, 0.959, Rotation2d.fromDegrees(60)), // Right
                new Pose2d(1.127, 7.2, Rotation2d.fromDegrees(-60)) // Left
        };
        private final static Pose2d[] blueRightReefPoints = new Pose2d[] {
                new Pose2d(2.95, 3.85, Rotation2d.fromDegrees(0)), // Pose A
                new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60)), // Pose C
                new Pose2d(5.44, 2.75, Rotation2d.fromDegrees(120)), // Pose E
                new Pose2d(6.08, 4.12, Rotation2d.fromDegrees(180)), // Pose G
                new Pose2d(5.13, 5.51, Rotation2d.fromDegrees(240)), // Pose I
                new Pose2d(3.54, 5.33, Rotation2d.fromDegrees(300)) // Pose K
        };
        private final static Pose2d[] blueLeftReefPoints = new Pose2d[] {
                new Pose2d(2.95, 4.12, Rotation2d.fromDegrees(0)), // Pose B
                new Pose2d(3.8, 2.59, Rotation2d.fromDegrees(60)), // Pose D
                new Pose2d(5.17, 2.61, Rotation2d.fromDegrees(120)), // Pose F
                new Pose2d(6.08, 3.78, Rotation2d.fromDegrees(180)), // Pose H
                new Pose2d(5.41, 5.35, Rotation2d.fromDegrees(240)), // Pose J
                new Pose2d(3.95, 5.53, Rotation2d.fromDegrees(300)) // Pose L
        };
        public final static Pose2d[] rightReefPoints = AllianceFlipUtil.apply(blueRightReefPoints);
        public final static Pose2d[] leftReefPoints = AllianceFlipUtil.apply(blueLeftReefPoints);
        public final static Pose2d[] coralStations = AllianceFlipUtil.apply(blueCoralStations);

        public static enum AutonState {
            RIGHTREEF,
            LEFTREEF,
            INTAKE,
            DEFAULT
        }

    }
}
