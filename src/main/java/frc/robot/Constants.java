// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

// import com.revrobotics.CANSparkBase.IdleMode;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopilotControllerPort = 1;
    public static final double kDeadband = 0.05;

  }

  public static final class MotorConstants {
    public static final int NewMotorId = 10; // Don't do 1-8! those are swerve
    public static final double speed = 0.5;
  }

  public static class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camera_0";
    public static String camera1Name = "camera_1";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0)); // TODO
                                                                                                               // update
    public static Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));// TODO
                                                                                                                   // update

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static class DriveConstants {
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(21);
    public static final double wheelBase = Units.inchesToMeters(25.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations = new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
    };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-Math.PI / 2);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(Math.PI);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(Math.PI / 2);

    // Device CAN IDs
    public static final int pigeonCanId = 9;

    public static final int frontLeftDriveCanId = 6;
    public static final int backLeftDriveCanId = 8;
    public static final int frontRightDriveCanId = 4;
    public static final int backRightDriveCanId = 2;

    public static final int frontLeftTurnCanId = 5;
    public static final int backLeftTurnCanId = 7;
    public static final int frontRightTurnCanId = 3;
    public static final int backRightTurnCanId = 1;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 50;
    public static final double wheelRadiusMeters = 0.0381;
    static final int kDrivingMotorPinionTeeth = 14;
    public static final double driveMotorReduction = (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0); // MAXSwerve
                                                                                                        // with 14
                                                                                                        // pinion teeth
                                                                                                        // and 22 spur
                                                                                                        // teeth
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1); // TODO update

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
                                                                                               // Wheel Radians
    public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM ->
                                                                                                        // Wheel Rad/Sec

    // Drive PID configuration
    public static final double driveKp = 0.04;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0; // TODO update
    public static final double driveKv = 0.1; // TODO update
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = true;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0; // TODO update
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);// TODO update

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    public static final double turnKp = 1; // TODO is this right?, was 2
    public static final double turnKd = 0.0;
    public static final double turnSimP = 1;// TODO is this right? was 8
    public static final double turnSimD = 0.0;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 74.088; // TODO update
    public static final double robotMOI = 6.883; // TODO update
    public static final double wheelCOF = 1.2; // TODO update
    public static final RobotConfig ppConfig = new RobotConfig(
        robotMassKg,
        robotMOI,
        new ModuleConfig(
            wheelRadiusMeters,
            maxSpeedMetersPerSec,
            wheelCOF,
            driveGearbox.withReduction(driveMotorReduction),
            driveMotorCurrentLimit,
            1),
        moduleTranslations);
  }
}
