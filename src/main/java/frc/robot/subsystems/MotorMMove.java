package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class MotorMMove extends SubsystemBase {
    private final SparkMax newMotor1;

    public MotorMMove() {
        newMotor1 = new SparkMax(MotorConstants.NewMotorId, MotorType.kBrushless);

    }

    public void setMotorSpeed(double speed) {
        newMotor1.set(speed);
    }
}
