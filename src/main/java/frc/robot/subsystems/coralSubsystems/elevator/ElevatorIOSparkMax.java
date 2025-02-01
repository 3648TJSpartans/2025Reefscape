// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.coralSubsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.AbsoluteEncoder;
import frc.robot.subsystems.coralSubsystems.coralConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSparkMax implements ElevatorIO {
    // declaration of the motors and encoders
    private final SparkMax motor;
    private final AbsoluteEncoder encoder;
    private PIDController pid;

    // constructor
    public ElevatorIOSparkMax() {
        motor = new SparkMax(coralConstants.coralElevator, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
        pid = new PIDController(coralConstants.kP, coralConstants.kI, coralConstants.kD);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.encoder
                .positionConversionFactor(coralConstants.elevatorEncoderPositionFactor)
                .velocityConversionFactor(coralConstants.elevatorEncoderPositionFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
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
    public void elevateTo(double setHeight) {
        motor.set(pid.calculate(getHeight(), setHeight));
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
