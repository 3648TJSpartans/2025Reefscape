// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralSubsystems;

import edu.wpi.first.math.kinematics.Odometry;

/** constants for the coral subsystem */
public class coralConstants {
    public static final double intakeSpeed = .5;
    // these are the ids for the CANs that will be use in the coral subsystem
    public static final int coralElevator = 9;
    public static final int coralWrist = 10;
    public static final int coralIntake = 11;
    // this is the constant for the pin of the ir sensor on the intake system of the
    // coral subsytem
    public static final int irSensorPin = 1;

    // these are the constants used for the encoder's pins
    public static final int levelChannelA = 2;
    public static final int levelChannelB = 3;
    public static final int angleChannelA = 4;
    public static final int angleChannelB = 5;

    public static final double coralLeveL1 = .1; // this is a momentary value that will be changed later
    public static final double coralLeveL2 = .2; // this is a momentary value that will be changed later
    public static final double coralLeveL3 = .3; // this is a momentary value that will be changed later
    public static final double coralLeveL4 = .4;
    // these are the constants for the elevator pid
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kF = 0;
    public static final double odometryFrequency = 100;
    public static final double kLiftMinRange = 0;
    public static final double kLiftMaxRange = 1;
    public static final boolean elevatorEncoderInverted = false;
    public static final double elevatorEncoderPositionFactor = 1;
    // these are the constants for the angle pid
    public static final double angle_kP = 1;
    public static final double angle_kI = 0;
    public static final double angle_kD = 0;

    // this is the angle for the wrist and is subject to change for each level
    public static final double anglevalue = 45;

}
