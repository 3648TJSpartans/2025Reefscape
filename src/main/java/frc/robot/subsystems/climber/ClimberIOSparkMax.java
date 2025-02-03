package frc.robot.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

public class ClimberIOSparkMax implements ClimberIO {

    private final SparkMax leadMotor;
    private final SparkMax followMotor;

    private final AbsoluteEncoder leadEncoder;
    private final SparkClosedLoopController leadController;

    public ClimberIOSparkMax() {
        leadMotor = new SparkMax(ClimberConstants.leadMotorID, MotorType.kBrushless);
        followMotor = new SparkMax(ClimberConstants.followMotorID, MotorType.kBrushless);
        leadEncoder = leadMotor.getAbsoluteEncoder();

        leadController = leadMotor.getClosedLoopController();

        SparkMaxConfig leadConfig = new SparkMaxConfig();
        leadConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(ClimberConstants.kLeadP, ClimberConstants.kLeadI, ClimberConstants.kLeadD,
                        ClimberConstants.kLeadFF)
                .outputRange(ClimberConstants.kLeadMinOutput, ClimberConstants.kLeadMaxOutput);
        leadConfig.inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0);
        leadConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ClimberConstants.leadOdometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        leadConfig.absoluteEncoder
                .inverted(ClimberConstants.leadEncoderInverted)
                .positionConversionFactor(ClimberConstants.leadEncoderPositionFactor)
                .velocityConversionFactor(ClimberConstants.leadEncoderPositionFactor)
                .averageDepth(2);
        leadMotor.configure(
                leadConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(ClimberConstants.kFollowP, ClimberConstants.kFollowI, ClimberConstants.kFollowD,
                        ClimberConstants.kFollowFF)
                .outputRange(ClimberConstants.kFollowMinOutput, ClimberConstants.kFollowMaxOutput);
        followConfig.inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0);
        followConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ClimberConstants.followOdometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        followConfig.absoluteEncoder
                .inverted(ClimberConstants.followEncoderInverted)
                .positionConversionFactor(ClimberConstants.followEncoderPositionFactor)
                .velocityConversionFactor(ClimberConstants.followEncoderPositionFactor)
                .averageDepth(2);
        // followConfig.follow(leadMotor);
        followMotor.configure(
                followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setPosition(double pos) {
        leadController.setReference(pos, ControlType.kPosition);
    }

    public void resetPosition() {
        leadController.setReference(0, ControlType.kPosition);
    }

    public void stop() {
        leadMotor.stopMotor();
        followMotor.stopMotor();
    }

    public double getPosition() {
        return leadEncoder.getPosition();
    }

    public void setSpeed(double speed) {
        leadMotor.set(speed);
        followMotor.set(speed);
    }
}
