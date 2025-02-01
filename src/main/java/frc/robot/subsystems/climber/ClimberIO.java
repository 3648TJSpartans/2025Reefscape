package frc.robot.subsystems.climber;

public interface ClimberIO {

    public default void setPosition(double pos) {
    }

    public default void resetPosition() {
    }

    public default void stop() {
    }

    public default double getPosition() {
        return 0;
    }

}
