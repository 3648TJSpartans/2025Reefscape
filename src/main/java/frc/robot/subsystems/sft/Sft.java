package frc.robot.subsystems.sft;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sft extends SubsystemBase {
    private final SftIO io;

    public Sft(SftIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateValues();
    }

    public double getAngle() {
        return io.getAngle();
    }

    public void rotateTo(double setangle) {
        io.rotateTo(setangle);
    }

    public void stopMotor() {
        io.stopMotor();
    }

    public boolean getIR() {
        return io.getIR();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }
}
