package frc.robot.subsystems.coralIntake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
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
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TunableNumber;

import com.revrobotics.spark.SparkClosedLoopController;

public class CoralIntakeIOSparkMax implements CoralIntakeIO {
    // declaration of motors, IR sensor and encoder
    private SparkMax wristMotor;
    private DigitalInput irSensor;
    private SparkMax intakeMotor;
    private AbsoluteEncoder absoluteEncoder;
    private SparkClosedLoopController wristController;

    // this is the constructor of the class
    public CoralIntakeIOSparkMax() {
        wristMotor = new SparkMax(CoralIntakeConstants.coralWrist, MotorType.kBrushless);
        intakeMotor = new SparkMax(CoralIntakeConstants.coralIntake, MotorType.kBrushless);
        irSensor = new DigitalInput(CoralIntakeConstants.irSensorPin);
        wristController = wristMotor.getClosedLoopController();
        absoluteEncoder = wristMotor.getAbsoluteEncoder();
        var wristConfig = new SparkMaxConfig();
        wristConfig.inverted(false)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12.0);
        wristConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(new LoggedTunableNumber("CoralIntake/kWristP", CoralIntakeConstants.kWristP).get(),
                        new LoggedTunableNumber("CoralIntake/kWristI", CoralIntakeConstants.kWristI).get(),
                        new LoggedTunableNumber("CoralIntake/kWristD", CoralIntakeConstants.kWristD).get(),
                        new LoggedTunableNumber("CoralIntake/kWristFF", CoralIntakeConstants.kWristFF).get())
                .outputRange(CoralIntakeConstants.kWristMinRange, CoralIntakeConstants.kWristMaxRange);
        wristConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / CoralIntakeConstants.wristOdometryFrequency))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        wristConfig.absoluteEncoder
                .inverted(CoralIntakeConstants.wristEncoderInverted)
                .positionConversionFactor(CoralIntakeConstants.wristEncoderPositionFactor)
                .velocityConversionFactor(CoralIntakeConstants.wristEncoderPositionFactor)
                .averageDepth(2);
        wristMotor.configure(
                wristConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void stopWristMotor() {
        wristMotor.set(0);
    }

    @Override
    public void stopIntakeMotor() {
        intakeMotor.set(0);
    }

    @Override
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void rotateTo(double setpoint) {
        wristController.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void updateValues() {
        Logger.recordOutput("Odometry/Sensor/coralIRsensorValue", getIR());
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
        wristMotor.set(speed);
    }
}
