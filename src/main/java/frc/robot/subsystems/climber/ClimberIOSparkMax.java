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

        private final SparkMax leftMotor;
        private final SparkMax rightMotor;

        private final AbsoluteEncoder leftEncoder;
        private final SparkClosedLoopController leftController;
        private final SparkClosedLoopController rightController;

        public ClimberIOSparkMax() {
                leftMotor = new SparkMax(ClimberConstants.leftMotorID, MotorType.kBrushless);
                rightMotor = new SparkMax(ClimberConstants.rightMotorID, MotorType.kBrushless);
                leftEncoder = leftMotor.getAbsoluteEncoder();

                leftController = leftMotor.getClosedLoopController();
                rightController = rightMotor.getClosedLoopController();

                SparkMaxConfig leftConfig = new SparkMaxConfig();
                leftConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .pidf(new TunableNumber("Climber/kleftP", ClimberConstants.kLeftP).get(),
                                                new TunableNumber("Climber/kleftI", ClimberConstants.kLeftI).get(),
                                                new TunableNumber("Climber/kleftD", ClimberConstants.kLeftD).get(),
                                                new TunableNumber("Climber/kleftFF", ClimberConstants.kLeftFF).get())
                                .outputRange(new TunableNumber("Climber/kleftMinOutput",
                                                ClimberConstants.kLeftMinOutput).get(),
                                                new TunableNumber("Climber/kLeftMaxOutput",
                                                                ClimberConstants.kLeftMaxOutput).get());
                leftConfig.inverted(false)
                                .idleMode(IdleMode.kBrake)
                                .voltageCompensation(12.0);
                leftConfig.signals
                                .absoluteEncoderPositionAlwaysOn(true)
                                .absoluteEncoderPositionPeriodMs(
                                                (int) (1000.0 / ClimberConstants.leftOdometryFrequency))
                                .absoluteEncoderVelocityAlwaysOn(true)
                                .absoluteEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);
                leftConfig.absoluteEncoder
                                .inverted(ClimberConstants.leftEncoderInverted)
                                .positionConversionFactor(ClimberConstants.leftEncoderPositionFactor)
                                .velocityConversionFactor(ClimberConstants.leftEncoderPositionFactor)
                                .averageDepth(2);
                leftMotor.configure(
                                leftConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                SparkMaxConfig rightConfig = new SparkMaxConfig();
                rightConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .pidf(new TunableNumber("Climber/kRightP", ClimberConstants.kRightP).get(),
                                                new TunableNumber("Climber/kRightI", ClimberConstants.kRightI).get(),
                                                new TunableNumber("Climber/kRightD", ClimberConstants.kRightD).get(),
                                                new TunableNumber("Climber/kRightFF", ClimberConstants.kRightFF)
                                                                .get())
                                .outputRange(new TunableNumber("Climber/kRightdMinOutput",
                                                ClimberConstants.kRightMinOutput).get(),
                                                new TunableNumber("Climber/kRightMaxOutput",
                                                                ClimberConstants.kRightMaxOutput).get());
                rightConfig.inverted(false)
                                .idleMode(IdleMode.kBrake)
                                .voltageCompensation(12.0);
                rightConfig.signals
                                .absoluteEncoderPositionAlwaysOn(true)
                                .absoluteEncoderPositionPeriodMs(
                                                (int) (1000.0 / ClimberConstants.rightOdometryFrequency))
                                .absoluteEncoderVelocityAlwaysOn(true)
                                .absoluteEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);
                rightConfig.absoluteEncoder
                                .inverted(ClimberConstants.rightEncoderInverted)
                                .positionConversionFactor(ClimberConstants.rightEncoderPositionFactor)
                                .velocityConversionFactor(ClimberConstants.rightEncoderPositionFactor)
                                .averageDepth(2);
                // rightConfig.follow(leftMotor);
                rightMotor.configure(
                                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        }

        public void setPosition(double posLeft, double posRight) {
                leftController.setReference(posLeft, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                rightController.setReference(posRight, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }

        public void resetPosition() {
                leftController.setReference(0, ControlType.kPosition);
                leftController.setReference(0, ControlType.kPosition);
        }

        public void stop() {
                leftMotor.stopMotor();
                rightMotor.stopMotor();
        }

        public double getPositions() {
                return leftEncoder.getPosition();
        }

        public void setSpeed(double speed) {
                leftMotor.set(speed);
                rightMotor.set(speed);
        }
}
