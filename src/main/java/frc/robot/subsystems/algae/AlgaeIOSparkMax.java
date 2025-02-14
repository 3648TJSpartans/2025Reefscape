package frc.robot.subsystems.algae;

import java.util.logging.LogManager;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumber;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeIOSparkMax implements AlgaeIO {

        private final SparkMax liftMotor;
        private final SparkMax intakeMotorLeft;
        private final SparkMax intakeMotorRight;
        private final AbsoluteEncoder liftEncoder;
        private final SparkClosedLoopController liftController;

        public AlgaeIOSparkMax() {
                // define motors and controllers
                liftMotor = new SparkMax(AlgaeConstants.liftMotorId, MotorType.kBrushless);
                intakeMotorLeft = new SparkMax(AlgaeConstants.intakeMotorLeftId, MotorType.kBrushless);
                intakeMotorRight = new SparkMax(AlgaeConstants.intakeMotorRightId, MotorType.kBrushless);
                liftEncoder = liftMotor.getAbsoluteEncoder();
                liftController = liftMotor.getClosedLoopController();

                // configure pid
                var liftConfig = new SparkMaxConfig();
                liftConfig.idleMode(IdleMode.kBrake);
                liftConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .pidf(new LoggedTunableNumber("Algae/kLiftP", AlgaeConstants.kLiftP).get(),
                                                new LoggedTunableNumber("Algae/kLiftI", AlgaeConstants.kLiftI).get(),
                                                new LoggedTunableNumber("Algae/kLiftD", AlgaeConstants.kLiftD).get(),
                                                new LoggedTunableNumber("Algae/kLiftFF", AlgaeConstants.kLiftFF).get())
                                .outputRange(new LoggedTunableNumber("Algae/kLiftMinRange",
                                                AlgaeConstants.kLiftMinRange)
                                                .get(),
                                                new LoggedTunableNumber("Algae/kLiftMaxRange",
                                                                AlgaeConstants.kLiftMaxRange)
                                                                .get());
                liftConfig.inverted(false)
                                .idleMode(IdleMode.kBrake)
                                .voltageCompensation(12.0);
                liftConfig.signals
                                .absoluteEncoderPositionAlwaysOn(true)
                                .absoluteEncoderPositionPeriodMs((int) (1000.0
                                                / new LoggedTunableNumber("Algae/liftOdometryFrequency",
                                                                AlgaeConstants.liftOdometryFrequency).get()))
                                .absoluteEncoderVelocityAlwaysOn(true)
                                .absoluteEncoderVelocityPeriodMs(20)
                                .appliedOutputPeriodMs(20)
                                .busVoltagePeriodMs(20)
                                .outputCurrentPeriodMs(20);
                liftConfig.absoluteEncoder
                                .inverted(AlgaeConstants.liftEncoderInverted)
                                .positionConversionFactor(
                                                new LoggedTunableNumber("Algae/liftEncoderPositionFactor",
                                                                AlgaeConstants.liftEncoderPositionFactor)
                                                                .get())
                                .velocityConversionFactor(
                                                new LoggedTunableNumber("Algae/liftEncoderVelocityFactor",
                                                                AlgaeConstants.liftEncoderVelocityFactor)
                                                                .get())
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

                intakeMotorLeft.set(speed);
                intakeMotorRight.set(speed);

        }

        public double getLiftPosition() {
                return liftEncoder.getPosition();
        }

        @Override
        public void updateValues() {
                Logger.recordOutput("Algae/EncoderPosition", liftEncoder.getPosition());
        }
}
