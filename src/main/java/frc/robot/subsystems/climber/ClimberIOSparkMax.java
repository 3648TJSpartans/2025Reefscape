package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkClosedLoopController;

public class ClimberIOSparkMax implements ClimberIO {

    private final SparkMax leadMotor;
    private final SparkMax followMotor;

    private final AbsoluteEncoder leadEncoder;
    private final AbsoluteEncoder followEncoder;
    private final SparkClosedLoopController leadController;
    private final SparkClosedLoopController followController;

    public ClimberIOSparkMax() {
        leadMotor = new SparkMax(ClimberConstants.leftMotorID, MotorType.kBrushless);
        followMotor = new SparkMax(ClimberConstants.rightMotorID, MotorType.kBrushless);
        leadEncoder = leadMotor.getAbsoluteEncoder();
        followEncoder = followMotor.getAbsoluteEncoder();

        var followConfig = new SparkMaxConfig();
        followConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ClimberConstants.kFollowP, ClimberConstants.kFollowI, ClimberConstants.kFollowD,
                        ClimberConstants.kFollowFF)
                .outputRange(ClimberConstants.kLeadP, ClimberConstants.kLeadP);
        followConfig.follow(leadMotor);
        followMotor.configure(
                followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leadController = leadMotor.getClosedLoopController();
        followController = followMotor.getClosedLoopController();
        var leadConfig = new SparkMaxConfig();
        leadConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(ClimberConstants.kLeadP, ClimberConstants.kLeadI, ClimberConstants.kLeadD,
                        ClimberConstants.kLeadFF)
                .outputRange(ClimberConstants.kLeadP, ClimberConstants.kLeadP);
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
    }
}
