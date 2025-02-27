package frc.robot.subsystems.sft;

public interface SftIO {
    public default double getAngle() {
        return 0;
    }

    public default void rotateTo(double setangle) {
    }

    public default void stopMotor() {
    }

    // update ir sensor state
    public default void updateValues() {

    }

    public default boolean getIR() {
        return false;
    }

    public default void setSpeed(double speed) {

    }
}
