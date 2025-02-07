// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralSubsystems;

import frc.robot.util.TunableNumber;

/** constants for the coral subsystem */
public class CoralConstants {
    public static final double intakeSpeed = .5;
    // these are the ids for the CANs that will be use in the coral subsystem
    public static final int coralElevator = 9;
    public static final int coralWrist = 10;
    public static final int coralIntake = 11;
    // this is the constant for the pin of the ir sensor on the intake system of the
    // coral subsytem
    public static final int irSensorPin = 0;

    // these are the constants used for the encoder's pins
    public static final int levelChannelA = 2;
    public static final int levelChannelB = 3;
    public static final int angleChannelA = 4;
    public static final int angleChannelB = 5;

    public static final double L1Height = 5; // this is a momentary value that will be changed later
    public static final double L2Height = 10; // this is a momentary value that will be changed later
    public static final double L3Height = 15; // this is a momentary value that will be changed later
    public static final double L4Height = 20;

    public static final double L1Angle = .4; // angle 0-1
    public static final double L2Angle = .5; // angle 0-1
    public static final double L3Angle = .6; // angle 0-1
    public static final double L4Angle = .7; // angle 0-1
    // these are the constants for the elevator pid
    public static final double kElevatorP = .01;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorFF = 0;
    public static final double odometryFrequency = 100;
    public static final double kElevatorMinRange = -.5;
    public static final double kElevatorMaxRange = .5;
    public static final boolean elevatorEncoderInverted = false;
    public static final double elevatorEncoderPositionFactor = 1;
    public static final double elevatorEncoderVelocityFactor = 1;

    public static final double kWristP = .5;
    public static final double kWristI = 0;
    public static final double kWristD = 0;
    public static final double kWristFF = 0;
    public static final double wristOdometryFrequency = 100;
    public static final double kWristMinRange = -1;
    public static final double kWristMaxRange = 1;
    public static final boolean wristEncoderInverted = false;
    public static final double wristEncoderPositionFactor = 1;

    // this is the angle for the wrist and is subject to change for each level
    public static final double anglevalue = 45;
    public static final int bottomLimitSwitchPin = (int) new TunableNumber("LimitSwitchPort", 2).get();

}
