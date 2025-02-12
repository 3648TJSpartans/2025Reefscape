package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    public void setPosition(double posLeft, double posRight) {
        io.setPosition(posLeft, posRight);
    }

    public void resetPosition() {
        io.resetPosition();
    }

    public double getPosition() {
        return io.getPosition();
    }

    public void setSpeed(double speed) {
        io.setSpeed(speed);
    }

}
