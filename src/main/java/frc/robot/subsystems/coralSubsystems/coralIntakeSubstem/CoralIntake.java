// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralSubsystems.coralIntakeSubstem;

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

  public void RotateTo(double setAngle) {
    io.RotateTo(setAngle);
  }

  public void takeIN() {
    io.takeIN();
  }

  public void takeOUT() {
    io.takeOUT();
  }
}
