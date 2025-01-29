package frc.robot.subsystems.Algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

    private final AlgaeIO io;

    public AlgaeSubsystem(AlgaeIO io) {
        this.io = io;
    }

    public void setLiftSpeed(double speed) {
        io.setLiftSpeed(speed);
    }

    public void setIntakeSpeed(double speed) {
        io.setIntakeSpeed(speed);
    }

    public double getLiftPosition() {
        return io.getLiftPosition();
    }
}
