package frc.robot.commands.goToCommands;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.util.AllianceFlipUtil;

public class AutonConstants {
        public static final double outtakeTime = .75;

        public static final double drivekP = 5.0;
        public static final double drivekD = 0.0;
        public static final double thetakP = 1.0;
        public static final double thetakD = 0.0;
        public static final double driveMaxVelocity = 1.5;
        public static final double driveMaxAcceleration = 1.5;
        public static final double thetaMaxVelocity = Units.degreesToRadians(360.0);
        public static final double thetaMaxAcceleration = Units.degreesToRadians(720.0);
        public static final double driveTolerance = 0.015;
        public static final double thetaTolerance = Units.degreesToRadians(1.5); // could be lower?
        public static final double ffMinRadius = 0.2;
        public static final double ffMaxRadius = 0.6;
        public static final int defaultLevel = 4;
        public static final double coralCutoff = .8;
        public static final double elevatorCutoff = 5;

        public static class PoseConstants {
                public final static Pose2d rightReef = new Pose2d(2.9, 3.73, Rotation2d.fromDegrees(0));
                public final static Pose2d leftReef = new Pose2d(2.9, 4.17, Rotation2d.fromDegrees(0));
                public final static Pose2d START = new Pose2d(2.85, 3.85, Rotation2d.fromDegrees(0));
                public final static double fieldLength = 17.548;
                public final static double fieldWidth = 8.042;

                private static Pose2d[] blueCoralStations = new Pose2d[] {
                                new Pose2d(1.11, 0.85, Rotation2d.fromDegrees(60)), // Right
                                new Pose2d(1.127, 7.2, Rotation2d.fromDegrees(-60)) // Left
                };

                private final static Pose2d[] l4ExactBlueRightReefPoints = new Pose2d[] {
                                new Pose2d(3.06, 3.78, Rotation2d.fromDegrees(0)), //

                                // HEY EVANGELIa UPDATE ALL OF OUR SETPOINTS USING ABOVE X AND Y VALUES!!! USE
                                // MICAH'S DESMOS GRAPH ON THE README

                                new Pose2d(3.97, 2.64, Rotation2d.fromDegrees(60)), // Pose D
                                new Pose2d(5.41, 2.86, Rotation2d.fromDegrees(120)), // Pose F
                                new Pose2d(5.94, 4.22, Rotation2d.fromDegrees(180)), // Pose H
                                new Pose2d(5.03, 5.36, Rotation2d.fromDegrees(240)), // Pose J
                                new Pose2d(3.59, 5.14, Rotation2d.fromDegrees(300)) // Pose L
                };

                private final static Pose2d[] l4ExactBlueLeftReefPoints = new Pose2d[] {
                                new Pose2d(3.11, 4.17, Rotation2d.fromDegrees(0)), // Pose B
                                new Pose2d(3.66, 2.89, Rotation2d.fromDegrees(60)), // Pose C
                                new Pose2d(5.05, 2.71, Rotation2d.fromDegrees(120)), // Pose E
                                new Pose2d(5.89, 3.83, Rotation2d.fromDegrees(180)), // Pose G
                                new Pose2d(5.34, 5.12, Rotation2d.fromDegrees(240)), // Pose I
                                new Pose2d(3.95, 5.29, Rotation2d.fromDegrees(300)) // Pose K
                };

                private final static Pose2d[] l2ExactBlueRightReefPoints = new Pose2d[] {
                                new Pose2d(2.9, 3.85, Rotation2d.fromDegrees(0)), // Pose A
                                new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60)), // Pose C
                                new Pose2d(5.44, 2.75, Rotation2d.fromDegrees(120)), // Pose E
                                new Pose2d(6.08, 4.12, Rotation2d.fromDegrees(180)), // Pose G
                                new Pose2d(5.13, 5.51, Rotation2d.fromDegrees(240)), // Pose I
                                new Pose2d(3.54, 5.33, Rotation2d.fromDegrees(300)) // Pose K
                };

                private final static Pose2d[] l2ExactBlueLeftReefPoints = new Pose2d[] {
                                new Pose2d(2.9, 4.12, Rotation2d.fromDegrees(0)), // Pose B
                                new Pose2d(3.8, 2.59, Rotation2d.fromDegrees(60)), // Pose D
                                new Pose2d(5.17, 2.61, Rotation2d.fromDegrees(120)), // Pose F
                                new Pose2d(6.08, 3.78, Rotation2d.fromDegrees(180)), // Pose H
                                new Pose2d(5.41, 5.35, Rotation2d.fromDegrees(240)), // Pose J
                                new Pose2d(3.95, 5.53, Rotation2d.fromDegrees(300)) // Pose L
                };

                private final static Pose2d[] l1ExactBlueRightReefPoints = new Pose2d[] {
                                new Pose2d(2.97, 3.79, Rotation2d.fromDegrees(0)), // Pose A
                                new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60)), // Pose C
                                new Pose2d(5.44, 2.75, Rotation2d.fromDegrees(120)), // Pose E
                                new Pose2d(6.08, 4.12, Rotation2d.fromDegrees(180)), // Pose G
                                new Pose2d(5.13, 5.51, Rotation2d.fromDegrees(240)), // Pose I
                                new Pose2d(3.54, 5.33, Rotation2d.fromDegrees(300)) // Pose K
                };

                private final static Pose2d[] l1ExactBlueLeftReefPoints = new Pose2d[] {
                                new Pose2d(2.97, 4.12, Rotation2d.fromDegrees(0)), // Pose B
                                new Pose2d(3.8, 2.59, Rotation2d.fromDegrees(60)), // Pose D
                                new Pose2d(5.17, 2.61, Rotation2d.fromDegrees(120)), // Pose F
                                new Pose2d(6.08, 3.78, Rotation2d.fromDegrees(180)), // Pose H
                                new Pose2d(5.41, 5.35, Rotation2d.fromDegrees(240)), // Pose J
                                new Pose2d(3.95, 5.53, Rotation2d.fromDegrees(300)) // Pose L
                };

                private final static Pose2d[] l4CloseBlueRightReefPoints = new Pose2d[] {
                                new Pose2d(2.97, 3.85, Rotation2d.fromDegrees(0)), // Pose A
                                new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60)), // Pose C
                                new Pose2d(5.39, 2.74, Rotation2d.fromDegrees(120)), // Pose E
                                new Pose2d(6.03, 4.15, Rotation2d.fromDegrees(180)), // Pose G
                                new Pose2d(5.14, 5.4, Rotation2d.fromDegrees(240)), // Pose I
                                new Pose2d(3.61, 5.25, Rotation2d.fromDegrees(300)) // Pose K
                };

                private final static Pose2d[] l4CloseBlueLeftReefPoints = new Pose2d[] {

                                new Pose2d(2.97, 4.15, Rotation2d.fromDegrees(0)), // Pose B
                                new Pose2d(3.31, 2.9, Rotation2d.fromDegrees(60)), // Pose D
                                new Pose2d(5.13, 2.59, Rotation2d.fromDegrees(120)), // Pose F
                                new Pose2d(6.03, 3.85, Rotation2d.fromDegrees(180)), // Pose H
                                new Pose2d(5.39, 5.25, Rotation2d.fromDegrees(240)), // Pose J
                                new Pose2d(3.86, 5.4, Rotation2d.fromDegrees(300)) // Pose L
                };

                private final static Pose2d[] l2CloseBlueRightReefPoints = new Pose2d[] {
                                new Pose2d(2.73, 3.85, Rotation2d.fromDegrees(0)), // Pose A
                                new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60)), // Pose C
                                new Pose2d(5.44, 2.75, Rotation2d.fromDegrees(120)), // Pose E
                                new Pose2d(6.08, 4.12, Rotation2d.fromDegrees(180)), // Pose G
                                new Pose2d(5.13, 5.51, Rotation2d.fromDegrees(240)), // Pose I
                                new Pose2d(3.54, 5.33, Rotation2d.fromDegrees(300)) // Pose K
                };

                private final static Pose2d[] l2CloseBlueLeftReefPoints = new Pose2d[] {
                                new Pose2d(2.73, 4.12, Rotation2d.fromDegrees(0)), // Pose B
                                new Pose2d(3.8, 2.59, Rotation2d.fromDegrees(60)), // Pose D
                                new Pose2d(5.17, 2.61, Rotation2d.fromDegrees(120)), // Pose F
                                new Pose2d(6.08, 3.78, Rotation2d.fromDegrees(180)), // Pose H
                                new Pose2d(5.41, 5.35, Rotation2d.fromDegrees(240)), // Pose J
                                new Pose2d(3.95, 5.53, Rotation2d.fromDegrees(300)) // Pose L
                };

                private final static Pose2d[] l1CloseBlueRightReefPoints = new Pose2d[] {
                                new Pose2d(2.97, 3.85, Rotation2d.fromDegrees(0)), // Pose A
                                new Pose2d(3.57, 2.75, Rotation2d.fromDegrees(60)), // Pose C
                                new Pose2d(5.44, 2.75, Rotation2d.fromDegrees(120)), // Pose E
                                new Pose2d(6.08, 4.12, Rotation2d.fromDegrees(180)), // Pose G
                                new Pose2d(5.13, 5.51, Rotation2d.fromDegrees(240)), // Pose I
                                new Pose2d(3.54, 5.33, Rotation2d.fromDegrees(300)) // Pose K
                };

                private final static Pose2d[] l1CloseBlueLeftReefPoints = new Pose2d[] {
                                new Pose2d(2.97, 4.12, Rotation2d.fromDegrees(0)), // Pose B
                                new Pose2d(3.8, 2.59, Rotation2d.fromDegrees(60)), // Pose D
                                new Pose2d(5.17, 2.61, Rotation2d.fromDegrees(120)), // Pose F
                                new Pose2d(6.08, 3.78, Rotation2d.fromDegrees(180)), // Pose H
                                new Pose2d(5.41, 5.35, Rotation2d.fromDegrees(240)), // Pose J
                                new Pose2d(3.95, 5.53, Rotation2d.fromDegrees(300)) // Pose L
                };

                private final static Pose2d[] blueCloseReefPoints = new Pose2d[] {
                                new Pose2d(3.14, 4, Rotation2d.fromDegrees(0)), // Pose B
                                // new Pose2d(3.8, 2.59, Rotation2d.fromDegrees(60)), // Pose D
                                // new Pose2d(5.17, 2.61, Rotation2d.fromDegrees(120)), // Pose F
                                // new Pose2d(6.08, 3.78, Rotation2d.fromDegrees(180)), // Pose H
                                // new Pose2d(5.41, 5.35, Rotation2d.fromDegrees(240)), // Pose J
                                // new Pose2d(3.95, 5.53, Rotation2d.fromDegrees(300)) // Pose L
                };
                private final static Pose2d[] blueFarReefPoints = new Pose2d[] {
                                new Pose2d(2.65, 4, Rotation2d.fromDegrees(0)), // Pose B
                                // new Pose2d(3.8, 2.59, Rotation2d.fromDegrees(60)), // Pose D
                                // new Pose2d(5.17, 2.61, Rotation2d.fromDegrees(120)), // Pose F
                                // new Pose2d(6.08, 3.78, Rotation2d.fromDegrees(180)), // Pose H
                                // new Pose2d(5.41, 5.35, Rotation2d.fromDegrees(240)), // Pose J
                                // new Pose2d(3.95, 5.53, Rotation2d.fromDegrees(300)) // Pose L
                };
                private final static Pose2d[] blueFarrerReefPoints = new Pose2d[] {
                                new Pose2d(2.45, 4, Rotation2d.fromDegrees(0)), // Pose B
                                // new Pose2d(3.8, 2.59, Rotation2d.fromDegrees(60)), // Pose D
                                // new Pose2d(5.17, 2.61, Rotation2d.fromDegrees(120)), // Pose F
                                // new Pose2d(6.08, 3.78, Rotation2d.fromDegrees(180)), // Pose H
                                // new Pose2d(5.41, 5.35, Rotation2d.fromDegrees(240)), // Pose J
                                // new Pose2d(3.95, 5.53, Rotation2d.fromDegrees(300)) // Pose L
                };
                public final static Pose2d[] l4ExactRightReefPoints = AllianceFlipUtil
                                .apply(l4ExactBlueRightReefPoints);
                public final static Pose2d[] l4ExactLeftReefPoints = AllianceFlipUtil.apply(l4ExactBlueLeftReefPoints);
                public final static Pose2d[] l2ExactRightReefPoints = AllianceFlipUtil
                                .apply(l2ExactBlueRightReefPoints);
                public final static Pose2d[] l2ExactLeftReefPoints = AllianceFlipUtil.apply(l2ExactBlueLeftReefPoints);
                public final static Pose2d[] l1ExactRightReefPoints = AllianceFlipUtil
                                .apply(l1ExactBlueRightReefPoints);
                public final static Pose2d[] l1ExactLeftReefPoints = AllianceFlipUtil.apply(l1ExactBlueLeftReefPoints);
                public final static Pose2d[] l4CloseRightReefPoints = AllianceFlipUtil
                                .apply(l4CloseBlueRightReefPoints);
                public final static Pose2d[] l4CloseLeftReefPoints = AllianceFlipUtil.apply(l4CloseBlueLeftReefPoints);
                public final static Pose2d[] l2CloseRightReefPoints = AllianceFlipUtil
                                .apply(l2CloseBlueRightReefPoints);
                public final static Pose2d[] l2CloseLeftReefPoints = AllianceFlipUtil.apply(l2CloseBlueLeftReefPoints);
                public final static Pose2d[] l1CloseRightReefPoints = AllianceFlipUtil
                                .apply(l1CloseBlueRightReefPoints);
                public final static Pose2d[] l1CloseLeftReefPoints = AllianceFlipUtil.apply(l1CloseBlueLeftReefPoints);
                public final static Pose2d[] closeReefPoints = AllianceFlipUtil.apply(blueCloseReefPoints);
                public final static Pose2d[] farReefPoints = AllianceFlipUtil.apply(blueFarReefPoints);
                public final static Pose2d[] farrerReefPoints = AllianceFlipUtil.apply(blueFarrerReefPoints);
                public final static Pose2d[] coralStations = AllianceFlipUtil.apply(blueCoralStations);

                public static enum AutonState {
                        RIGHTREEF,
                        LEFTREEF,
                        INTAKE,
                        DEFAULT
                }

        }
}
