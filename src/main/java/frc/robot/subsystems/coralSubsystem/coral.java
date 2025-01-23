// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralSubsystem;

//import of some usefull modules 
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

// coral subsystem class
public class coral extends SubsystemBase {
  /** declaration of some essential motors */
  private SparkMax LevelMotor = new SparkMax(coralConstants.coralCANID1, MotorType.kBrushless);
  private SparkMax takInOutMotor = new SparkMax(coralConstants.coralCANID2, MotorType.kBrushless);
  private SparkMax AngleMotor = new SparkMax(coralConstants.coralCANID3, MotorType.kBrushless);
  // declaration of the IR sensor
  private DigitalInput irSensor = new DigitalInput(coralConstants.irSensorPin);

  public coral() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // this function allows us to elevate the elevator up to a certain level
  public void elevate(double level) {

  }

  // this function stop the level motor
  public void stopLevelM() {
    LevelMotor.set(0);
  }

  // this function stop the motor that take in and out the coral
  public void stopTakingM() {
    takInOutMotor.set(0);
  }

  // this method stop the motor that control the angle of the coral depositor
  public void stopAngleM() {
    AngleMotor.set(0);
  }

  // this method allows us to taking in a coral
  public void takeIN() {
    if (irSensor.get()) {
      takInOutMotor.set(0);
    } else if (!irSensor.get()) {
      takInOutMotor.set(1);// im not sure if this is 1 or -1 depending of the sense of rotation of the
                           // mechanism
    }
  }

  // this method allows us to expulse a coral
  public void takeOut() {

  }
}
