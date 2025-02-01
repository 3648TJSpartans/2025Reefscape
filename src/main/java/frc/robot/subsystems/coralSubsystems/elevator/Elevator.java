// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralSubsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.ModuleIOInputsAutoLogged;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public class Elevator extends SubsystemBase {
  // declaration of a instance
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Creates a new elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateValues();
  }

  public void elevateTo(double setHeight) {
    io.elevateTo(setHeight);
  }

  // allows us to stop the motor
  public void stop() {
    io.stop();
  }

  // allows us to know the heigh of the elevator in encoders unit
  public double getHeight() {
    return io.getHeight();
  }

  // allows us to reset the encoder value
  public void resetEncoder() {
    io.resetEncoder();
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

}
