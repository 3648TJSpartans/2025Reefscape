// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralSubsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.coralSubsystems.coralConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
    // declaration of the motors and encoders
    private final SparkMax motor;
    private final Encoder encoder;
    private PIDController pid;

    // constructor
    public ElevatorIOSparkMax() {
        motor = new SparkMax(coralConstants.coralElevator, MotorType.kBrushless);
        encoder = new Encoder(coralConstants.levelChannelA, coralConstants.levelChannelB, false,
                Encoder.EncodingType.k4X);
        pid = new PIDController(coralConstants.kP, coralConstants.kI, coralConstants.kD);
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public double getHeight() {
        return encoder.getDistance();
    }

    @Override
    public void elevateTo(double setHeight) {
        motor.set(pid.calculate(getHeight(), setHeight));
    }

    @Override
    public void resetEncoder() {
        encoder.reset();
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
