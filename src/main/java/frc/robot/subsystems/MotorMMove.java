package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;


public class MotorMMove extends SubsystemBase {
    private final CANSparkMax newMotor1;

    public MotorMMove() {
        newMotor1 = new CANSparkMax(MotorConstants.NewMotorId, MotorType.kBrushless);

    }

    public void setMotorSpeed(double speed) {
        newMotor1.set(speed);
    }
}

