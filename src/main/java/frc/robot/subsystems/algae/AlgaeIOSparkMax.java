package frc.robot.subsystems.algae;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.coralSubsystems.CoralConstants;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeIOSparkMax implements AlgaeIO {

    private final SparkMax liftMotor;
    private final SparkMax intakeMotor;
    private final AbsoluteEncoder liftEncoder;
    private final SparkClosedLoopController liftController;

    public AlgaeIOSparkMax() {
        // define motors and controllers
        liftMotor = new SparkMax(AlgaeConstants.liftMotorId, MotorType.kBrushless);
        intakeMotor = new SparkMax(AlgaeConstants.intakeMotorId, MotorType.kBrushless);
        liftEncoder = liftMotor.getAbsoluteEncoder();
        liftController = liftMotor.getClosedLoopController();

        // configure pid
        var liftConfig = new SparkMaxConfig();
        liftConfig.idleMode(IdleMode.kBrake);
        liftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(AlgaeConstants.kLiftP, AlgaeConstants.kLiftI, AlgaeConstants.kLiftD, AlgaeConstants.kLiftFF)
                .outputRange(AlgaeConstants.kLiftMinRange, AlgaeConstants.kLiftMaxRange);
        liftConfig.inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0);
        liftConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / AlgaeConstants.liftOdometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        liftConfig.absoluteEncoder
                .inverted(AlgaeConstants.liftEncoderInverted)
                .positionConversionFactor(AlgaeConstants.liftEncoderPositionFactor)
                .velocityConversionFactor(AlgaeConstants.liftEncoderVelocityFactor)
                .averageDepth(2);
        liftMotor.configure(
                liftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    // shouldn't be needed
    public void setLiftSpeed(double speed) {
        liftMotor.set(speed);
    }

    public void setLiftPosition(double position) {
        liftController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public double getLiftPosition() {
        return liftEncoder.getPosition();
    }
}
