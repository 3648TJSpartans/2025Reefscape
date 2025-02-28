package frc.robot.subsystems.sft;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.coralIntake.CoralIntakeConstants;
import frc.robot.util.TunableNumber;

import com.revrobotics.spark.SparkClosedLoopController;

public class SftIOSparkMax implements SftIO {
    // declaration of motors, IR sensor and encoder
    private SparkMax motor;
    private DigitalInput irSensor;
    private RelativeEncoder encoder;
    private AbsoluteEncoder absoluteEncoder;
    private SparkClosedLoopController motorController;

    // this is the constructor of the class
    public SftIOSparkMax() {
        motor = new SparkMax(CoralIntakeConstants.coralWrist, MotorType.kBrushless);
        irSensor = new DigitalInput(CoralIntakeConstants.irSensorPin);
        absoluteEncoder = motor.getAbsoluteEncoder();
        var config = new SparkMaxConfig();
        config.inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(new TunableNumber("CoralIntake/kWristP", CoralIntakeConstants.kWristP).get(),
                        new TunableNumber("CoralIntake/kWristI", CoralIntakeConstants.kWristI).get(),
                        new TunableNumber("CoralIntake/kWristD", CoralIntakeConstants.kWristD).get(),
                        new TunableNumber("CoralIntake/kWristFF", CoralIntakeConstants.kWristFF).get())
                .outputRange(CoralIntakeConstants.kWristMinRange, CoralIntakeConstants.kWristMaxRange);
        config.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / CoralIntakeConstants.wristOdometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        config.absoluteEncoder
                .inverted(CoralIntakeConstants.wristEncoderInverted)
                .positionConversionFactor(CoralIntakeConstants.wristEncoderPositionFactor)
                .velocityConversionFactor(CoralIntakeConstants.wristEncoderPositionFactor)
                .averageDepth(2);
        motor.configure(
                config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void stopMotor() {
        motor.set(0);
    }

    @Override
    public void rotateTo(double setpoint) {
        motorController.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void updateValues() {
        Logger.recordOutput("SFT/IRSensorValue", getIR());
    }

    @Override
    public double getAngle() {
        return absoluteEncoder.getPosition();
    }

    @Override
    public boolean getIR() {
        return irSensor.get();
    }

    public void setWristSpeed(double speed) {
        motor.set(speed);
    }
}
