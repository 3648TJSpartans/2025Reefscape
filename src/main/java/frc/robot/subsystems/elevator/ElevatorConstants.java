// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/** constants for the coral subsystem */
public class ElevatorConstants {
    // these are the ids for the CANs that will be use in the coral subsystem
    public static final int coralElevator = 9;

    public static final double coralLeveL1 = 1; // this is a momentary value that will be changed later
    public static final double coralLeveL2 = 0.1; // this is a momentary value that will be changed later
    public static final double coralLeveL3 = 13.3; // this is a momentary value that will be changed later
    public static final double coralLeveL4 = 15;
    public static final double intakePose = 2.262;// this is the postion for the intake
    // these are the constants for the elevator pid
    public static final double kElevatorP = 1;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorFF = 0;
    public static final double odometryFrequency = 100;
    public static final double kElevatorMinRange = -.45;
    public static final double kElevatorMaxRange = .45;
    public static final boolean elevatorEncoderInverted = false;
    public static final double elevatorEncoderPositionFactor = 1;
    public static final double elevatorEncoderVelocityFactor = 1;
    public static final int bottomLimitSwitchPin = 2;
}
