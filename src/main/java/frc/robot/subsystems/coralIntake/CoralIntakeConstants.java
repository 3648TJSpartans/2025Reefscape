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
    public static final double kWristP = 1.8;
    public static final double kWristI = 0;
    public static final double kWristD = 1.5;// Was 2.5
    // Values estimated from @link
    public static final double kWristS = 0; // Volts- Static Gain
    public static final double kWristG = 0.41; // Volts - Gravity feedforward constant
    public static final double kWristV = 5.85; // V*s/rad - Velocity Feedforward Constant
    public static final double kWristA = 0.41; // V*s^2/rad - Acceleration Feedforward Constant

    public static final double kWristFF = 0;
    public static final double straightUpAngle = .42;
    public static final double wristOdometryFrequency = 100;
    public static final double kWristMinRange = -0.2;
    public static final double kWristMaxRange = 0.2;
    public static final boolean wristEncoderInverted = false;
    public static final double wristEncoderPositionFactor = 1;
    // this is the angle for the wrist and is subject to change for each level
    public static final double anglevalue = 45;

    public static final double L1Angle = .08;
    public static final double L2Angle = .145;
    public static final double L3Angle = .28;
    public static final double L4Angle = .30;
    public static final double IntakeAngle = .84;
    public static final double preIntakeAngle = .75;
    public static final double defaultAngle = .6; // TODO: change
    public static final double endgameAngle = .06;
    public static final double upAngle = .42;
    public static final double algaeRemovalAngle = .05;
    public static final double marginOfErrorIntake = .001;
    public static final double minAngle = 0.1;
    public static final double maxAngle = 0.95;
    public static final double coralHomeCutoff = .78;
    public static final double slamAngle = 0.06;
    public static final double marginOfError = 0.05; // this is how close the robot has to be to target rotation to
    // stop,
    // needs tuning

}
