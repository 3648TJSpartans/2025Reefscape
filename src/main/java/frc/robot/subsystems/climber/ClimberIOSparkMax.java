package frc.robot.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.util.TunableNumber;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

public class ClimberIOSparkMax implements ClimberIO {

        private final SparkMax leadMotor;
        private final SparkMax followMotor;

        private final AbsoluteEncoder leadEncoder;
        private final AbsoluteEncoder followEncoder;
        private final SparkClosedLoopController leadController;
        private final SparkClosedLoopController followController;

        public ClimberIOSparkMax() {
                leadMotor = new SparkMax(ClimberConstants.leadMotorID, MotorType.kBrushless);
                followMotor = new SparkMax(ClimberConstants.followMotorID, MotorType.kBrushless);
                leadEncoder = leadMotor.getAbsoluteEncoder();
                followEncoder = followMotor.getAbsoluteEncoder();

                leadController = leadMotor.getClosedLoopController();
                followController = followMotor.getClosedLoopController();

                SparkMaxConfig leadConfig = new SparkMaxConfig();
                leadConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .pidf(new TunableNumber("Climber/kLeadP", ClimberConstants.kLeadP).get(),
                                                new TunableNumber("Climber/kLeadI", ClimberConstants.kLeadI)
                                                                .get(),
                                                new TunableNumber("Climber/kLeadD", ClimberConstants.kLeadD)
                                                                .get(),
                                                new TunableNumber("Climber/kLeadFF", ClimberConstants.kLeadFF)
                                                                .get())
                                .outputRange(new TunableNumber("Climber/kLeadMinOutput",
                                                ClimberConstants.kLeadMinOutput).get(),
                                                new TunableNumber("Climber/kLeadMaxOutput",
                                                                ClimberConstants.kLeadMaxOutput).get());
                leadConfig.inverted(true)
                                .idleMode(IdleMode.kBrake)
                                .voltageCompensation(12.0);
                leadConfig.signals
                                .absoluteEncoderPositionAlwaysOn(true)
                                .absoluteEncoderPositionPeriodMs(
                                                (int) (1000.0 / ClimberConstants.leadOdometryFrequency))
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
                                .pidf(new TunableNumber("Climber/kFollowP", ClimberConstants.kFollowP).get(),
                                                new TunableNumber("Climber/kFollowI", ClimberConstants.kFollowI)
                                                                .get(),
                                                new TunableNumber("Climber/kFollowD", ClimberConstants.kFollowD)
                                                                .get(),
                                                new TunableNumber("Climber/kFollowFF", ClimberConstants.kFollowFF)
                                                                .get())
                                .outputRange(new TunableNumber("Climber/kFollowdMinOutput",
                                                ClimberConstants.kFollowMinOutput).get(),
                                                new TunableNumber("Climber/kFollowMaxOutput",
                                                                ClimberConstants.kFollowMaxOutput).get());
                followConfig.inverted(false)
                                .idleMode(IdleMode.kBrake)
                                .voltageCompensation(12.0);
                followConfig.signals
                                .absoluteEncoderPositionAlwaysOn(true)
                                .absoluteEncoderPositionPeriodMs(
                                                (int) (1000.0 / ClimberConstants.followOdometryFrequency))
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

        public void setPosition(double position1, double position2) {
                leadController.setReference(position1, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                followController.setReference(position2, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        public void resetPosition() {
                leadController.setReference(0, ControlType.kPosition);
        }

        public void stop() {
                leadMotor.stopMotor();
                followMotor.stopMotor();
        }

        public double getPosition() {
                return followEncoder.getPosition();
        }

        public void setSpeed(double speed) {
                leadMotor.set(speed);
                followMotor.set(speed);
        }
}
