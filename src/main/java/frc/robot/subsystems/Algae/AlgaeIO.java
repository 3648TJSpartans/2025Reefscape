package frc.robot.subsystems.Algae;

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
}
