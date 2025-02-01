// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralSubsystem;

//import of some usefull modules 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

// coral subsystem class
public class coral extends SubsystemBase {
  /** declaration of some essential motors */
  private SparkMax levelMotor = new SparkMax(coralConstants.coralCANID1, MotorType.kBrushless);
  private SparkMax takeInOutMotor = new SparkMax(coralConstants.coralCANID2, MotorType.kBrushless);
  private SparkMax angleMotor = new SparkMax(coralConstants.coralCANID3, MotorType.kBrushless);
  // declaration of the IR sensor
  private DigitalInput irSensor = new DigitalInput(coralConstants.irSensorPin);
  // declaration of the encoders
  private Encoder levelEncoder = new Encoder(coralConstants.levelChannelA, coralConstants.levelChannelB, false,
      Encoder.EncodingType.k4X);
  private Encoder angleEncoder = new Encoder(coralConstants.angleChannelA, coralConstants.angleChannelB, false,
      Encoder.EncodingType.k4X);
  private PIDController levelPID = new PIDController(coralConstants.kP, coralConstants.kI, coralConstants.kD);
  private PIDController anglePID = new PIDController(coralConstants.angle_kP, coralConstants.angle_kI,
      coralConstants.angle_kD);

  public coral() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // this method stop the level motor
  public void stopLevelM() {
    levelMotor.set(0);
  }

  // this method stop the motor that take in and out the coral
  public void stopTakingM() {
    takeInOutMotor.set(0);
  }

  // this method stop the motor that control the angle of the coral depositor
  public void stopAngleM() {
    angleMotor.set(0);
  }

  // this method allows the coral subsystem to elevate itself up to a certain
  // height
  public void elevateTo(double setHeight) {
    levelMotor.set(levelPID.calculate(getHeight(), setHeight));
  }

  // this method allows the the elevator's claw to rotate up to a certain angle
  public void RotateTo(double setpoint) {
    angleMotor.set(anglePID.calculate(getAngle(), setpoint));
  }

  // this method allows us to taking in a coral and stop when the sensor detects
  // it
  public void takeIN() {
    if (irSensor.get()) {
      takeInOutMotor.set(0);
    } else if (!irSensor.get()) {
      takeInOutMotor.set(1);// im not sure if this is 1 or -1 depending of the sense of rotation of the
                            // mechanism
    }
  }

  // this method allows us to reset the encoder used to know the level of the
  // elevator
  public void resetLevelEn() {
    levelEncoder.reset();
  }

  // this method allows us to rest the angle encoder
  public void resetAngleEn() {
    angleEncoder.reset();
  }

  // this method allows us to know how height the coral subssytem have been
  // elevated
  public double getHeight() {
    return levelEncoder.getDistance();
  }

  // this method allows us to know the angle of the intake system in the coral
  // subsytem
  public double getAngle() {
    return angleEncoder.getDistance();
  }

}
