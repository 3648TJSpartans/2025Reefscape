// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  // declaration of a instance
  private final ElevatorIO io;

  /** Creates a new elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateValues();
    Logger.recordOutput("Elevator/3dSetpoint", new Pose3d(0, io.getHeightMeters(), 0, new Rotation3d()));
    io.updateLimitSwitch();
  }

  public boolean getBottomLimitSwitch() {
    return io.getBottomLimitSwitch();
  }

  public boolean getTopLimitSwitch() {
    return io.getTopLimitSwitch();
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

  public boolean atBottom() {
    return io.atBottom();
  }

  public void zeroElevator() {
    io.zeroElevator();
  }

  public boolean getLimitReset() {
    return io.getLimitReset();
  }
}
