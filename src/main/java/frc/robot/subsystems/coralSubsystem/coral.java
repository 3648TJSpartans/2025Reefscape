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
  private SparkMax LevelMotor = new SparkMax(coralConstants.coralCANID1, MotorType.kBrushless);
  private SparkMax takInOutMotor = new SparkMax(coralConstants.coralCANID2, MotorType.kBrushless);
  private SparkMax AngleMotor = new SparkMax(coralConstants.coralCANID3, MotorType.kBrushless);
  // declaration of the IR sensor
  private DigitalInput irSensor = new DigitalInput(coralConstants.irSensorPin);
  // declaration of the encoders
  private Encoder LevelEncoder = new Encoder(coralConstants.levelChannelA, coralConstants.levelChannelB, false,
      Encoder.EncodingType.k4X);
  private Encoder angleEncoder = new Encoder(coralConstants.angleChannelA, coralConstants.angleChannelB, false,
      Encoder.EncodingType.k4X);
  private PIDController LevelPID = new PIDController(coralConstants.kP, coralConstants.kI, coralConstants.kD);
  private PIDController AnglePID = new PIDController(coralConstants.angle_kP, coralConstants.angle_kI,
      coralConstants.angle_kD);

  public coral() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // this function stop the level motor
  public void stopLevelM() {
    LevelMotor.set(0);
  }

  // if somebody see this dont forget to adjust the setting of the level method
  // this function stop the motor that take in and out the coral
  public void stopTakingM() {
    takInOutMotor.set(0);
  }

  // this method stop the motor that control the angle of the coral depositor
  public void stopAngleM() {
    AngleMotor.set(0);
  }

  // this function allows the coral subsystem to elevate itself up to a certai
  // height
  public void elevateTo(double setHeight) {
    LevelMotor.set(LevelPID.calculate(GetHeight(), setHeight));
  }

  public void RotateTo(double setpoint) {
    AngleMotor.set(AnglePID.calculate(GetAngle(), setpoint));
  }

  // this method allows us to taking in a coral and stop when the sensor detects
  // it
  public void takeIN() {
    if (irSensor.get()) {
      takInOutMotor.set(0);
    } else if (!irSensor.get()) {
      takInOutMotor.set(1);// im not sure if this is 1 or -1 depending of the sense of rotation of the
                           // mechanism
    }
  }

  // this method allows us to reset the encoder
  public void resetLevelEn() {
    LevelEncoder.reset();
  }

  // this method allows us to rest the angle encoder
  public void resetAngleEn() {
    angleEncoder.reset();
  }

  // this method allows us to know how height the coral subssytem have been
  // elevated
  public double GetHeight() {
    return LevelEncoder.getDistance();
  }

  // this method allows us to know the angle of the intake system in the coral
  // subsytem
  public double GetAngle() {
    return angleEncoder.getDistance();
  }

}
