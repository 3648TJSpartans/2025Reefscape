package frc.robot.subsystems.coralSubsystems.CoralIntake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.coralSubsystems.coralConstants;

public class CoralIntakeIOSparkMax implements CoralIntakeIO {
    // declaration of motors, IR sensor and encoder
    private SparkMax wristMotor;
    private DigitalInput irSensor;
    private SparkMax intakeMotor;
    private Encoder encoder;
    private PIDController pid;

    // this is the constructor of the class
    public CoralIntakeIOSparkMax() {
        wristMotor = new SparkMax(coralConstants.coralCANID3, MotorType.kBrushless);
        intakeMotor = new SparkMax(coralConstants.coralCANID2, MotorType.kBrushless);
        irSensor = new DigitalInput(coralConstants.irSensorPin);
        encoder = new Encoder(coralConstants.angleChannelA, coralConstants.angleChannelB, false,
                Encoder.EncodingType.k4X);
        pid = new PIDController(coralConstants.angle_kP, coralConstants.angle_kI,
                coralConstants.angle_kD);
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
    public void takeIN() {
        if (irSensor.get()) {
            intakeMotor.set(0);
        } else if (!irSensor.get()) {
            intakeMotor.set(1);
        } // im not sure if this is 1 or -1 depending of the sense of rotation of the
          // mechanism
    }

    @Override
    public void takeOUT() {
        if (irSensor.get()) {
            intakeMotor.set(-1);
        } else if (!irSensor.get()) {
            intakeMotor.set(0);
        } // im not sure if this is 1 or -1 depending of the sense of rotation of the
          // mechanism
    }

    @Override
    public void RotateTo(double setpoint) {
        wristMotor.set(pid.calculate(getAngle(), setpoint));
    }

    @Override
    public double getAngle() {
        return encoder.getDistance();
    }
}
