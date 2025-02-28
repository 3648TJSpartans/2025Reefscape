// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import frc.robot.subsystems.coralIntake.CoralIntake;

public class Elevator extends SubsystemBase {
  // declaration of a instance
  private final ElevatorIO io;
  private CoralIntake m_coralIntake;

  /** Creates a new elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateValues();
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

  public boolean isInNoGoZone() {
    double noGoBottom = new TunableNumber("Elevator/NoGoZoneBottom", ElevatorConstants.NoGoZoneBottom).get();
    double noGoTop = new TunableNumber("Elevator/NoGoZoneTop", ElevatorConstants.NoGoZoneTop).get();
    double currentHeight = io.getHeight();
    return currentHeight > noGoBottom && currentHeight < noGoTop;
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
