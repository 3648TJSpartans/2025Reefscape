package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants.AlgaeConstants;
//import edu.wpi.first.*;

public class AlgaeSubsystem extends SubsystemBase {

    private final SparkMax liftMotor;
    private final SparkMax intakeMotor;
    private final AbsoluteEncoder liftEncoder;
    private final SparkClosedLoopController liftController;
    // private final

    public AlgaeSubsystem() {
        liftMotor = new SparkMax(AlgaeConstants.liftMotorId, MotorType.kBrushless);
        intakeMotor = new SparkMax(AlgaeConstants.intakeMotorId, MotorType.kBrushless);
        liftEncoder = liftMotor.getAbsoluteEncoder();
        liftController = liftMotor.getClosedLoopController();
        var liftConfig = new SparkMaxConfig();
        liftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(AlgaeConstants.kLiftP, AlgaeConstants.kLiftI, AlgaeConstants.kLiftD, AlgaeConstants.kLiftFF)
                .outputRange(AlgaeConstants.kLiftMinRange, AlgaeConstants.kLiftMaxRange);
    }

    public void setLiftSpeed(double speed) {
        // if (speed > 0) {
        // liftController.setReference(1, ControlType.kPosition);
        // }
        liftMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public double getLiftPosition() {
        return liftEncoder.getPosition();
    }
}
