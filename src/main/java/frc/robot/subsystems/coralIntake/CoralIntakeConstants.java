// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

/** constants for the coralintake subsystem */
public class CoralIntakeConstants {
    public static final double intakeSpeed = .2;
    public static final double outtakeSpeed = -.5;
    // these are the ids for the CANs that will be use in the coral subsystem
    public static final int coralElevator = 9;
    public static final int coralWrist = 10;
    public static final int coralIntake = 11;
    // this is the constant for the pin of the ir sensor on the intake system of the
    // coralintake subsytem
    public static final int irSensorPin = 1;
    // these are the constants for the pid of the robot's wrist
    public static final double kWristP = 0.6;
    public static final double kWristI = 0;
    public static final double kWristD = 0.1;
    public static final double kWristFF = 0;
    public static final double wristOdometryFrequency = 100;
    public static final double kWristMinRange = -1;
    public static final double kWristMaxRange = 1;
    public static final boolean wristEncoderInverted = false;
    public static final double wristEncoderPositionFactor = 1;
    // this is the angle for the wrist and is subject to change for each level
    public static final double anglevalue = 45;

    public static final double L1Angle = .08;
    public static final double L2Angle = .14;
    public static final double L3Angle = .28;
    public static final double L4Angle = .34;
    public static final double IntakeAngle = .723;
    public static final double defaultAngle = .6; // TODO: change
    public static final double algaeRemovalAngle = .14;

    public static final double minAngle = 0.1;
    public static final double maxAngle = 0.95;

    public static final double slamAngle = 0.06;
    public static final double marginOfError = 0.05; // this is how close the robot has to be to target rotation to
    // stop,
    // needs tuning

}
