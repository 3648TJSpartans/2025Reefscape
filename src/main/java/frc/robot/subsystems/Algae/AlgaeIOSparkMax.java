package frc.robot.subsystems.Algae;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;

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
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(AlgaeConstants.kLiftP, AlgaeConstants.kLiftI, AlgaeConstants.kLiftD, AlgaeConstants.kLiftFF)
                .outputRange(AlgaeConstants.kLiftMinRange, AlgaeConstants.kLiftMaxRange);
    }

    // shouldn't be needed
    public void setLiftSpeed(double speed) {
        liftMotor.set(speed);
    }

    public void setLiftPosition(double position) {
        liftController.setReference(position, ControlType.kPosition);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public double getLiftPosition() {
        return liftEncoder.getPosition();
    }
}
