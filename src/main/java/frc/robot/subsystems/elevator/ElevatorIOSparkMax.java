// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.util.TunableNumber;
import frc.robot.util.TunableNumber;

public class ElevatorIOSparkMax implements ElevatorIO {
    // declaration of the motors and encoders
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private PIDController pid;
    private final SparkClosedLoopController motorController;
    private boolean limitReset;
    private final DigitalInput bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchPin);
    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchPin);

    // constructor
    public ElevatorIOSparkMax() {
        motor = new SparkMax(ElevatorConstants.coralElevator, MotorType.kBrushless);
        motorController = motor.getClosedLoopController();
        limitReset = false;
        Logger.recordOutput("Elevator/EncoderReset", false);
        Logger.recordOutput("Elevator/Setpoint", 0.0);
        encoder = motor.getEncoder();
        var motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kBrake);// idle mode here!
        motorConfig.inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0);
        motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(new TunableNumber("Elevator/kElevatorP", ElevatorConstants.kElevatorP).get(),
                        new TunableNumber("Elevator/kElevatorI", ElevatorConstants.kElevatorI).get(),
                        new TunableNumber("Elevator/kElevatorD", ElevatorConstants.kElevatorD).get(),
                        new TunableNumber("Elevator/kElevatorFF", ElevatorConstants.kElevatorFF).get())
                .outputRange(
                        new TunableNumber("Elevator/kElevatorMinRange", ElevatorConstants.kElevatorMinRange)
                                .get(),
                        new TunableNumber("Elevator/kElevatorMaxRange", ElevatorConstants.kElevatorMaxRange)
                                .get());
        motorConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ElevatorConstants.odometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        motorConfig.absoluteEncoder
                .inverted(ElevatorConstants.elevatorEncoderInverted)
                .positionConversionFactor(new TunableNumber("Elevator/elevatorEncoderPositionFactor",
                        ElevatorConstants.elevatorEncoderPositionFactor).get())
                .velocityConversionFactor(ElevatorConstants.elevatorEncoderPositionFactor)
                .averageDepth(2);
        motorConfig.softLimit
                .reverseSoftLimit(0)
                // .forwardSoftLimit(ElevatorConstants.coralLimit);
                .forwardSoftLimit(50);
        motor.configure(
                motorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
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
        if (limitReset) {
            Logger.recordOutput("Elevator/Setpoint", position);
            motorController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }

    @Override
    public void resetEncoder() {
        // encoder.reset();
    }

    public void setSpeed(double speed) {
        Logger.recordOutput("Elevator/setSpeed", speed);
        motor.set(speed);
    }

    @Override
    public void updateValues() {
        Logger.recordOutput("Elevator/EncoderValue", encoder.getPosition());
    }

    @Override
    public void updateLimitSwitch() {
        Logger.recordOutput("Elevator/BottomLimitSwitchPushed", !bottomLimitSwitch.get());
        Logger.recordOutput("Elevator/TopLimitSwitchPushed", !topLimitSwitch.get());
        if (bottomLimitSwitch.get()) {
            setZero();
        } else if (topLimitSwitch.get()) {
            setTop();
        }
    }

    @Override
    public boolean getBottomLimitSwitch() {
        return !bottomLimitSwitch.get();
    }

    @Override
    public boolean getTopLimitSwitch() {
        return !topLimitSwitch.get();
    }

    private void setZero() {
        encoder.setPosition(0);
        limitReset = true;
        Logger.recordOutput("Elevator/EncoderReset", limitReset);
    }

    private void setTop() {
        encoder.setPosition(ElevatorConstants.coralLimit);
        limitReset = true;
        Logger.recordOutput("Elevator/EncoderReset", limitReset);
    }

    @Override
    public boolean atBottom() {
        return bottomLimitSwitch.get();
    }

    @Override
    public void zeroElevator() {
        setZero();
    }

    @Override
    public boolean getLimitReset() {
        return limitReset;
    }

    @AutoLogOutput(key = "Elevator/getSpeed")
    @Override
    public double getSpeed() {
        return motor.get();
    }
}
