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
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void RotateTo(double setpoint) {
        wristMotor.set(pid.calculate(getAngle(), setpoint));
    }

    @Override
    public double getAngle() {
        return encoder.getDistance();
    }

    @Override
    public boolean getIR() {
        return irSensor.get();
    }

    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
    }
}
