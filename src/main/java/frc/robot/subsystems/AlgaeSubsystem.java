package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.AbsoluteEncoder;
//import edu.wpi.first.*;

public class AlgaeSubsystem extends SubsystemBase {

    private final SparkMax liftMotor;
    private final SparkMax intakeMotor;
    private final AbsoluteEncoder liftEncoder;

    // private final

    public AlgaeSubsystem() {
        liftMotor = new SparkMax(9, MotorType.kBrushless);
        intakeMotor = new SparkMax(10, MotorType.kBrushless);
        liftEncoder = liftMotor.getAbsoluteEncoder();
        // liftPIDController = liftMotor.
    }

    public void setLiftSpeed(double speed) {
        liftMotor.set(speed);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public double getLiftPosition() {
        return liftEncoder.getPosition();
    }
}
