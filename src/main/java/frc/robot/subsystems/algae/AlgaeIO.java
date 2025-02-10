package frc.robot.subsystems.algae;

public interface AlgaeIO {

    public default void setLiftSpeed(double speed) {
    }

    public default void setLiftPosition(double position) {
    }

    public default void setIntakeSpeed(double speed) {
    }

    public default double getLiftPosition() {
        return 0;
    }

    public default boolean getIR() {
        return false;
    }
}
