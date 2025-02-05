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

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.subsystems.coralSubsystems.CoralConstants;
import frc.robot.util.TunableNumber;

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
        motorController = motor.getClosedLoopController();
        var motorConfig = new SparkMaxConfig();
        motorConfig.inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0);
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(new TunableNumber("ELevator/kElevatorP", CoralConstants.kElevatorP).get(),
                        new TunableNumber("ELevator/kElevatorI", CoralConstants.kElevatorI).get(),
                        new TunableNumber("ELevator/kElevatorD", CoralConstants.kElevatorD).get(),
                        new TunableNumber("ELevator/kElevatorFF", CoralConstants.kElevatorFF).get())
                .outputRange(new TunableNumber("Elevator/kElevatorMinRange", CoralConstants.kElevatorMinRange).get(),
                        new TunableNumber("Elevator/kElevatorMaxRange", CoralConstants.kElevatorMaxRange).get());
        motorConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / CoralConstants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        motorConfig.absoluteEncoder
                .inverted(CoralConstants.elevatorEncoderInverted)
                .positionConversionFactor(new TunableNumber("Elevator/elevatorEncoderPositionFactor",
                        CoralConstants.elevatorEncoderPositionFactor).get())
                .velocityConversionFactor(CoralConstants.elevatorEncoderPositionFactor)
                .averageDepth(2);
        motor.configure(
                motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        encoder = motor.getAbsoluteEncoder();
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
