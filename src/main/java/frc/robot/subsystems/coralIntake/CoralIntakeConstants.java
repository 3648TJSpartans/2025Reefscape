// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

/** constants for the coralintake subsystem */
public class CoralIntakeConstants {
    public static final double intakeSpeed = .2;
    // these are the ids for the CANs that will be use in the coral subsystem
    public static final int coralElevator = 9;
    public static final int coralWrist = 10;
    public static final int coralIntake = 11;
    // this is the constant for the pin of the ir sensor on the intake system of the
    // coralintake subsytem
    public static final int irSensorPin = 0;
    // these are the constants for the pid of the robot's wrist
    public static final double kWristP = 1;
    public static final double kWristI = 0;
    public static final double kWristD = 0;
    public static final double kWristFF = 0;
    public static final double wristOdometryFrequency = 100;
    public static final double kWristMinRange = -.5;
    public static final double kWristMaxRange = .5;
    public static final boolean wristEncoderInverted = false;
    public static final double wristEncoderPositionFactor = 1;
    // this is the angle for the wrist and is subject to change for each level
    public static final double anglevalue = 45;
    public static final double L1Angle = .1;
    public static final double L2Angle = .2;
    public static final double L3Angle = .28;
    public static final double L4Angle = .4;
    public static final double IntakeAngle = .723;

    public static final double marginOfError = 0.05; // this is how close the robot has to be to target rotation to
    // stop,
    // needs tuning

}
