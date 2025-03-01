package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    public void setPosition(double position1, double position2) {
        Logger.recordOutput("Climber/setPosition1", position1);
        Logger.recordOutput("Climber/setPosition2", position2);
        io.setPosition(position1, position2);
    }

    public void resetPosition() {
        Logger.recordOutput("Climber/resetPosition", true);
        io.resetPosition();
    }

    public double getPosition() {
        Logger.recordOutput("Climber/getPosition", io.getPosition());
        return io.getPosition();
    }

    public void setSpeed(double speed) {
        Logger.recordOutput("Climber/setSpeed", speed);
        io.setSpeed(speed);
    }

    public void stop() {
        io.stop();
    }

}
