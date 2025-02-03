// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralSubsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.AbsoluteEncoder;

import frc.robot.subsystems.algae.AlgaeConstants;
import frc.robot.subsystems.coralSubsystems.CoralConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSparkMax implements ElevatorIO {
    // declaration of the motors and encoders
    private final SparkMax motor;
    private final AbsoluteEncoder encoder;
    private PIDController pid;
    private final SparkClosedLoopController motorController;

    // constructor
    public ElevatorIOSparkMax() {
        motor = new SparkMax(CoralConstants.coralElevator, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        motorController = motor.getClosedLoopController();
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(CoralConstants.kP, CoralConstants.kD, CoralConstants.kI, CoralConstants.kF)
                .outputRange(AlgaeConstants.kLiftMinRange, AlgaeConstants.kLiftMaxRange);

    }

    @Override
    public void stop() {
        motor.set(0);
    }

    @Override
    public double getHeight() {
        return encoder.getPosition();
    }

    @Override
    public void elevateTo(double position) {
        motorController.setReference(position, ControlType.kPosition);
    }

    @Override
    public void resetEncoder() {
        // encoder.reset();
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void updateValues() {
        Logger.recordOutput("Odometry/Elevator/EncoderValue", encoder.getPosition());
    }

}
