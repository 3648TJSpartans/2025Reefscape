package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    public void setPosition(double position1, double position2) {
        io.setPosition(position1, position2);
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

    public void stop() {
        io.stop();
    }

}
