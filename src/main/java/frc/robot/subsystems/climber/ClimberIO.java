package frc.robot.subsystems.climber;

public interface ClimberIO {

    public default void setPosition(double position1, double position2) {
    }

    public default void resetPosition() {
    }

    public default void stop() {
    }

    public default double getPosition() {
        return 0;
    }

    public default void setSpeed(double speed) {
    }

}
