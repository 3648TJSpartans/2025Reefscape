// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.goToCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MaximizeAction;

import org.littletonrobotics.junction.Logger;

public class DriveToNearest extends Command {
        private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
        private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
        private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
        private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
        private static final LoggedTunableNumber driveMaxVelocity = new LoggedTunableNumber(
                        "DriveToPose/DriveMaxVelocity");
        private static final LoggedTunableNumber driveMaxVelocitySlow = new LoggedTunableNumber(
                        "DriveToPose/DriveMaxVelocitySlow");
        private static final LoggedTunableNumber driveMaxAcceleration = new LoggedTunableNumber(
                        "DriveToPose/DriveMaxAcceleration");
        private static final LoggedTunableNumber thetaMaxVelocity = new LoggedTunableNumber(
                        "DriveToPose/ThetaMaxVelocity");
        private static final LoggedTunableNumber thetaMaxAcceleration = new LoggedTunableNumber(
                        "DriveToPose/ThetaMaxAcceleration");
        private static final LoggedTunableNumber driveTolerance = new LoggedTunableNumber("DriveToPose/DriveTolerance");
        private static final LoggedTunableNumber thetaTolerance = new LoggedTunableNumber("DriveToPose/ThetaTolerance");
        private static final LoggedTunableNumber ffMinRadius = new LoggedTunableNumber("DriveToPose/FFMinRadius");
        private static final LoggedTunableNumber ffMaxRadius = new LoggedTunableNumber("DriveToPose/FFMinRadius");

        static {
                drivekP.initDefault(AutonConstants.drivekP);
                drivekD.initDefault(AutonConstants.drivekD);
                thetakP.initDefault(AutonConstants.thetakP);
                thetakD.initDefault(AutonConstants.thetakD);
                driveMaxVelocity.initDefault(AutonConstants.driveMaxVelocity);
                driveMaxAcceleration.initDefault(AutonConstants.driveMaxAcceleration);
                thetaMaxVelocity.initDefault(AutonConstants.thetaMaxVelocity);
                thetaMaxAcceleration.initDefault(AutonConstants.thetaMaxAcceleration);
                driveTolerance.initDefault(AutonConstants.driveTolerance);
                thetaTolerance.initDefault(AutonConstants.thetaTolerance);
                ffMinRadius.initDefault(AutonConstants.ffMinRadius);
                ffMaxRadius.initDefault(AutonConstants.ffMaxRadius);
        }

        private final Drive drive;
        private final Supplier<Pose2d> target;
        private final Supplier<Pose2d[]> targetPoints;
        private final ProfiledPIDController driveController = new ProfiledPIDController(
                        0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
        private final ProfiledPIDController thetaController = new ProfiledPIDController(
                        0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);

        private Translation2d lastSetpointTranslation = new Translation2d();
        private double driveErrorAbs = 0.0;
        private double thetaErrorAbs = 0.0;
        private boolean running = false;
        private Supplier<Pose2d> robot;

        private Supplier<Translation2d> linearFF = () -> Translation2d.kZero;
        private DoubleSupplier omegaFF = () -> 0.0;

        public DriveToNearest(Drive drive, Supplier<Pose2d[]> targets) {
                this.drive = drive;
                robot = drive::getPose;
                this.targetPoints = targets;
                this.target = () -> nearestPoint(robot, targets);
                // Enable continuous input for theta controller
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                addRequirements(drive);
        }

        public DriveToNearest(Drive drive, Supplier<Pose2d[]> targets, Supplier<Pose2d> robot) {
                this(drive, targets);
                this.robot = robot;
        }

        public DriveToNearest(
                        Drive drive,
                        Supplier<Pose2d[]> targets,
                        Supplier<Pose2d> robot,
                        Supplier<Translation2d> linearFF,
                        DoubleSupplier omegaFF) {
                this(drive, targets, robot);
                this.linearFF = linearFF;
                this.omegaFF = omegaFF;
        }

        @Override
        public void initialize() {
                Pose2d currentPose = robot.get();
                ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
                Translation2d linearFieldVelocity = new Translation2d(fieldVelocity.vxMetersPerSecond,
                                fieldVelocity.vyMetersPerSecond);
                driveController.reset(
                                currentPose.getTranslation().getDistance(target.get().getTranslation()),
                                Math.min(
                                                0.0,
                                                -linearFieldVelocity
                                                                .rotateBy(
                                                                                target
                                                                                                .get()
                                                                                                .getTranslation()
                                                                                                .minus(currentPose
                                                                                                                .getTranslation())
                                                                                                .getAngle()
                                                                                                .unaryMinus())
                                                                .getX()));
                thetaController.reset(
                                currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
                lastSetpointTranslation = currentPose.getTranslation();
        }

        @Override
        public void execute() {
                running = true;

                // Update from tunable numbers
                if (driveMaxVelocity.hasChanged(hashCode())
                                || driveMaxVelocitySlow.hasChanged(hashCode())
                                || driveMaxAcceleration.hasChanged(hashCode())
                                || driveTolerance.hasChanged(hashCode())
                                || thetaMaxVelocity.hasChanged(hashCode())
                                || thetaMaxAcceleration.hasChanged(hashCode())
                                || thetaTolerance.hasChanged(hashCode())
                                || drivekP.hasChanged(hashCode())
                                || drivekD.hasChanged(hashCode())
                                || thetakP.hasChanged(hashCode())
                                || thetakD.hasChanged(hashCode())) {
                        driveController.setP(drivekP.get());
                        driveController.setD(drivekD.get());
                        driveController.setConstraints(
                                        new TrapezoidProfile.Constraints(driveMaxVelocity.get(),
                                                        driveMaxAcceleration.get()));
                        driveController.setTolerance(driveTolerance.get());
                        thetaController.setP(thetakP.get());
                        thetaController.setD(thetakD.get());
                        thetaController.setConstraints(
                                        new TrapezoidProfile.Constraints(thetaMaxVelocity.get(),
                                                        thetaMaxAcceleration.get()));
                        thetaController.setTolerance(thetaTolerance.get());
                }

                // Get current pose and target pose
                Pose2d currentPose = robot.get();
                Pose2d targetPose = target.get();

                // Calculate drive speed
                double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                double ffScaler = MathUtil.clamp(
                                (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                                0.0,
                                1.0);
                driveErrorAbs = currentDistance;
                driveController.reset(
                                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                                driveController.getSetpoint().velocity);
                double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                                + driveController.calculate(driveErrorAbs, 0.0);
                if (currentDistance < driveController.getPositionTolerance())
                        driveVelocityScalar = 0.0;
                lastSetpointTranslation = new Pose2d(
                                targetPose.getTranslation(),
                                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                                .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
                                .getTranslation();

                // Calculate theta speed
                double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                                + thetaController.calculate(
                                                currentPose.getRotation().getRadians(),
                                                targetPose.getRotation().getRadians());
                thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
                if (thetaErrorAbs < thetaController.getPositionTolerance())
                        thetaVelocity = 0.0;

                Translation2d driveVelocity = new Pose2d(
                                new Translation2d(),
                                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                                .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
                                .getTranslation();

                // Scale feedback velocities by input ff
                final double linearS = linearFF.get().getNorm();
                final double thetaS = omegaFF.getAsDouble();
                driveVelocity = driveVelocity.interpolate(linearFF.get().times(driveMaxVelocity.get()), linearS);
                thetaVelocity = MathUtil.interpolate(
                                thetaVelocity, omegaFF.getAsDouble() * thetaMaxVelocity.get(), thetaS);

                // Command speeds
                drive.runVelocity(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity,
                                                currentPose.getRotation()));

                // Log data
                Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
                Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
                Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
                Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
                Logger.recordOutput(
                                "DriveToPose/Setpoint",
                                new Pose2d(
                                                lastSetpointTranslation,
                                                Rotation2d.fromRadians(thetaController.getSetpoint().position)));
                Logger.recordOutput("DriveToPose/Goal", targetPose);
        }

        @Override
        public void end(boolean interrupted) {
                drive.stop();
                running = false;
                // Clear logs
                Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
                Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
        }

        /** Checks if the robot is stopped at the final pose. */
        public boolean atGoal() {
                return running && driveController.atGoal() && thetaController.atGoal();
        }

        /**
         * Checks if the robot pose is within the allowed drive and theta tolerances.
         */
        public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
                return running
                                && Math.abs(driveErrorAbs) < driveTolerance
                                && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
        }

        public Pose2d nearestPoint(Supplier<Pose2d> robotPose, Supplier<Pose2d[]> targetPoints) {
                Pose2d[] points = targetPoints.get();
                Pose2d drivePose = robotPose.get();
                Pose2d targetPose = points[0];
                double minDistance = Double.MAX_VALUE;
                for (Pose2d point : points) {
                        double distance = drivePose.getTranslation().getDistance(point.getTranslation());
                        if (distance < minDistance) {
                                minDistance = distance;
                                targetPose = point;
                        }
                }
                return targetPose;

        }
}
