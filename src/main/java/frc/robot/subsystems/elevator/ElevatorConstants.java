// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/** constants for the coral subsystem */
public class ElevatorConstants {
    // these are the ids for the CANs that will be use in the coral subsystem
    public static final int coralElevator = 9;

    public static final double coralLeveL1 = 0; // this is a momentary value that will be changed later
    public static final double coralLeveL2 = 0.15; // this is a momentary value that will be changed later
    public static final double coralLeveL3 = 0.8; // this is a momentary value that will be changed later
    public static final double coralLeveL4 = 73;
    public static final double coralLimit = 74.8;
    public static final double intakePose = 0;// this is the postion for the intake
    public static final double defaultPosition = 0.5; // TODO: change this to the correct number
    // these are the constants for the elevator pid
    public static final double kElevatorP = .3;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0.1;
    public static final double kElevatorFF = 0.1;
    public static final double odometryFrequency = 100;
    public static final double kElevatorMinRange = -.45;
    public static final double kElevatorMaxRange = .45;
    public static final boolean elevatorEncoderInverted = false;
    public static final double elevatorEncoderPositionFactor = 1;
    public static final double elevatorEncoderVelocityFactor = 1;
    public static final int bottomLimitSwitchPin = 2;

    public static final double marginOfError = 1; // this is how close the robot has to be to target height to stop,
                                                  // needs tuning

}
