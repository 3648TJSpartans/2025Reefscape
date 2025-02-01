package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    public void setPosition(double pos) {
        io.setPosition(pos);
    }

    public void resetPosition() {
        io.resetPosition();
    }

    public double getPosition() {
        return io.getPosition();
    }

}
