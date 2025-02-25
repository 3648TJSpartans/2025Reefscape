// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  // declaration of some usefull instance
  CoralIntakeIO io;

  /** Creates a new CoralIntake. */
  public CoralIntake(CoralIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateValues();
    Logger.recordOutput("Intake/EncoderAngle", getAngle());
    Logger.recordOutput("Intake/IR", getIR());

  }

  public void stopWristMotor() {
    io.stopWristMotor();
  }

  public void stopIntakeMotor() {
    io.stopIntakeMotor();
  }

  public double getAngle() {
    return io.getAngle();
  }

  public void rotateTo(double setAngle) {
    Logger.recordOutput("Intake/setAngle", setAngle);
    if (CoralIntakeConstants.minAngle < setAngle && setAngle < CoralIntakeConstants.maxAngle) {
      io.rotateTo(setAngle);
    }
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public boolean getIR() {
    return io.getIR();
  }

  public void setWristSpeed(double speed) {
    // if ((CoralIntakeConstants.minAngle < getAngle() || speed >= 0)
    // && (CoralIntakeConstants.minAngle > getAngle() || speed <= 0)) {
    // io.setWristSpeed(speed);
    // }
    io.setWristSpeed(speed);
  }
}
