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
import frc.robot.util.TunableNumber;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MaximizeAction;

import org.littletonrobotics.junction.Logger;

public class DriveToNearest4 extends Command {
        private static final TunableNumber drivekP = new TunableNumber("DriveToPose/DrivekP");
        private static final TunableNumber drivekD = new TunableNumber("DriveToPose/DrivekD");
        private static final TunableNumber thetakP = new TunableNumber("DriveToPose/ThetakP");
        private static final TunableNumber thetakD = new TunableNumber("DriveToPose/ThetakD");
        private static final TunableNumber driveMaxVelocity = new TunableNumber(
                        "DriveToPose/DriveMaxVelocity");
        private static final TunableNumber driveMaxVelocitySlow = new TunableNumber(
                        "DriveToPose/DriveMaxVelocitySlow");
        private static final TunableNumber driveMaxAcceleration = new TunableNumber(
                        "DriveToPose/DriveMaxAcceleration");
        private static final TunableNumber thetaMaxVelocity = new TunableNumber(
                        "DriveToPose/ThetaMaxVelocity");
        private static final TunableNumber thetaMaxAcceleration = new TunableNumber(
                        "DriveToPose/ThetaMaxAcceleration");
        private static final TunableNumber driveTolerance = new TunableNumber("DriveToPose/DriveTolerance");
        private static final TunableNumber thetaTolerance = new TunableNumber("DriveToPose/ThetaTolerance");
        private static final TunableNumber ffMinRadius = new TunableNumber("DriveToPose/FFMinRadius");
        private static final TunableNumber ffMaxRadius = new TunableNumber("DriveToPose/FFMinRadius");

        static {
                drivekP.setDefault(AutonConstants.drivekP);
                drivekD.setDefault(AutonConstants.drivekD);
                thetakP.setDefault(AutonConstants.thetakP);
                thetakD.setDefault(AutonConstants.thetakD);
                driveMaxVelocity.setDefault(AutonConstants.driveMaxVelocity);
                driveMaxAcceleration.setDefault(AutonConstants.driveMaxAcceleration);
                thetaMaxVelocity.setDefault(AutonConstants.thetaMaxVelocity);
                thetaMaxAcceleration.setDefault(AutonConstants.thetaMaxAcceleration);
                driveTolerance.setDefault(AutonConstants.driveTolerance);
                thetaTolerance.setDefault(AutonConstants.thetaTolerance);
                ffMinRadius.setDefault(AutonConstants.ffMinRadius);
                ffMaxRadius.setDefault(AutonConstants.ffMaxRadius);
        }

        private final Drive drive;
        private Supplier<Pose2d> target;
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

        public DriveToNearest4(Drive drive, Supplier<Pose2d[]> targets) {
                this.drive = drive;
                robot = drive::getPose;
                this.targetPoints = targets;
                this.target = () -> nearestPoint(robot, targets);
                // Enable continuous input for theta controller
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                addRequirements(drive);
        }

        public DriveToNearest4(Drive drive, Supplier<Pose2d[]> targets, Supplier<Pose2d> robot) {
                this(drive, targets);
                this.robot = robot;
        }

        public DriveToNearest4(
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
                // ChassisSpeeds fieldVelocity = drive.getChassisSpeeds();
                ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(),
                                drive.getRotation());
                System.out.println("fieldVelocity: " + fieldVelocity.toString());
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
                System.out.println("running: " + running);

                // Update from tunable numbers
                if (driveMaxVelocity.hasChanged()
                                || driveMaxVelocitySlow.hasChanged()
                                || driveMaxAcceleration.hasChanged()
                                || driveTolerance.hasChanged()
                                || thetaMaxVelocity.hasChanged()
                                || thetaMaxAcceleration.hasChanged()
                                || thetaTolerance.hasChanged()
                                || drivekP.hasChanged()
                                || drivekD.hasChanged()
                                || thetakP.hasChanged()
                                || thetakD.hasChanged()) {
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
                        System.out.println("vars changed");
                } else {
                        System.out.println("vars didn't changed");
                }

                Logger.recordOutput("DriveToPose/varIsChanged", false);
                Logger.recordOutput("DriveToPose/running", running);

                // Get current pose and target pose
                Pose2d currentPose = robot.get();
                Pose2d targetPose = target.get();
                System.out.println("currentPose: " + currentPose.toString());
                System.out.println("targetPose: " + targetPose.toString());

                // Calculate drive speed
                double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                System.out.println("currentDistance: " + currentDistance);
                double ffScaler = MathUtil.clamp(
                                (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
                                0.0,
                                1.0);
                System.out.println("ffScaler: " + ffScaler);
                Logger.recordOutput("DriveToPose/ffScaler", ffScaler);
                driveErrorAbs = currentDistance;

                System.out.println("driveErrorAbs: " + driveErrorAbs);

                System.out.println("BEFORE DRIVECONTROLLER RESET");

                System.out.println("driveController setpoint: " + driveController.getSetpoint().position);
                System.out.println("driveVelocityScalar 1:" + driveController.getSetpoint().velocity);
                System.out.println("driveVelocityScalar 2: " + driveController.calculate(driveErrorAbs, 0.0));
                driveController.reset(
                                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                                driveController.getSetpoint().velocity);
                System.out.println("driverController is reset");
                double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                                + driveController.calculate(driveErrorAbs, 0.0);
                System.out.println("driveController setpoint: " + driveController.getSetpoint().position);
                System.out.println("driveVelocityScalar 1:" + driveController.getSetpoint().velocity);
                System.out.println("driveVelocityScalar 2: " + driveController.calculate(driveErrorAbs, 0.0));
                System.out.println("driveVelocityScalar: " + driveVelocityScalar);

                if (currentDistance < driveController.getPositionTolerance())
                        driveVelocityScalar = 0.0;
                System.out.println("driveVelocityScalar after tolerance: " + driveVelocityScalar);

                Logger.recordOutput("DriveToPose/driveVelocityScalar", driveVelocityScalar);
                lastSetpointTranslation = new Pose2d(
                                targetPose.getTranslation(),
                                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                                .transformBy(GeomUtil.toTransform2d(driveController.getSetpoint().position, 0.0))
                                .getTranslation();
                System.out.println("lastSetpointTranslation: " + lastSetpointTranslation.toString());

                Logger.recordOutput("DriveToPose/lastSetpointTranslation", lastSetpointTranslation);

                // Calculate theta speed
                double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                                + thetaController.calculate(
                                                currentPose.getRotation().getRadians(),
                                                targetPose.getRotation().getRadians());

                System.out.println("thetaVelocity: " + thetaVelocity);

                thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
                if (thetaErrorAbs < thetaController.getPositionTolerance())
                        thetaVelocity = 0.0;

                System.out.println("thetaVelocity after tolerance: " + thetaVelocity);

                Logger.recordOutput("DriveToPose/thetaErrorAbs", thetaErrorAbs);

                Translation2d driveVelocity = new Pose2d(
                                new Translation2d(),
                                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                                .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
                                .getTranslation();

                System.out.println("driveVelocity: " + driveVelocity.toString());

                // Scale feedback velocities by input ff
                final double linearS = linearFF.get().getNorm();
                final double thetaS = omegaFF.getAsDouble();
                driveVelocity = driveVelocity.interpolate(linearFF.get().times(driveMaxVelocity.get()), linearS);
                System.out.println("driveVelocity after interpolation: " + driveVelocity);

                thetaVelocity = MathUtil.interpolate(
                                thetaVelocity, omegaFF.getAsDouble() * thetaMaxVelocity.get(), thetaS);
                System.out.println("thetaVelocity after interpolation: " + thetaVelocity);

                // Command speeds
                drive.runVelocity(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity,
                                                currentPose.getRotation()));
                System.out.println("runVelocity ran");

                // Log data
                Logger.recordOutput("DriveToPose/driveVelocity", driveVelocity);
                Logger.recordOutput("DriveToPose/thetaVelocity", thetaVelocity);
                Logger.recordOutput("DriveToPose/DistanceMeasured", currentDistance);
                Logger.recordOutput("DriveToPose/IsFinished", isFinished());
                Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
                Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
                Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
                Logger.recordOutput(
                                "DriveToPose/Setpoint",
                                new Pose2d(
                                                lastSetpointTranslation,
                                                Rotation2d.fromRadians(thetaController.getSetpoint().position)));
                Logger.recordOutput("DriveToPose/Goal", targetPose);
                Logger.recordOutput("DriveToPose/Trajectory", currentPose, targetPose);

        }

        @Override
        public void end(boolean interrupted) {
                drive.stop();
                running = false;
                // Clear logs
                Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
                Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
                System.out.println("!!!!!!!!!!!!!!!DriveToNearest finished!!!!!!!!!!!!!");
        }

        /** Checks if the robot is stopped at the final pose. */
        public boolean atGoal() {
                System.out.println("atGoal ran");
                return running && driveController.atGoal() && thetaController.atGoal();

        }

        public Pose2d finalPose(Supplier<Pose2d> robotPose) {
                Pose2d drivePose = robotPose.get();
                return drivePose;
        }

        @Override
        public boolean isFinished() {
                System.out.println("isFinished ran. atGoal:" + atGoal());
                return atGoal();
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
                System.out.println("nearestPoint ran");
                return targetPose;

        }
}
