package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

    private final AlgaeIO io;

    public AlgaeSubsystem(AlgaeIO io) {
        this.io = io;
        Logger.recordOutput("Subsystems/Algae/Setpoint", 0);
    }

    public void setLiftSpeed(double speed) {
        io.setLiftSpeed(speed);
    }

    public void setLiftPosition(double pos) {
        Logger.recordOutput("Subsystems/Algae/Setpoint", pos);
        io.setLiftPosition(pos);
    }

    public void setIntakeSpeed(double speed) {
        io.setIntakeSpeed(speed);
    }

    public double getLiftPosition() {
        return io.getLiftPosition();
    }

    @Override
    public void periodic() {
        io.updateValues();
    }
}
