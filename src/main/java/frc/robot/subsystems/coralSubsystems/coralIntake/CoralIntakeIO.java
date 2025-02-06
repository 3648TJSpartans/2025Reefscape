package frc.robot.subsystems.coralSubsystems.coralIntake;

public interface CoralIntakeIO {
    public default double getAngle() {
        return 0;
    }

    // allows us to rotate the arm of the coral subsystem to a given angle
    public default void rotateTo(double setangle) {
    }

    // allows us to stop the wrist motor
    public default void stopWristMotor() {
    }

    // allows us to stop the intake motor
    public default void stopIntakeMotor() {
    }

    // sets speed
    public default void setSpeed(double speed) {
    }

    // update ir sensor state
    public default void updateValues() {

    }

    // Gets IR Sensor state
    public default boolean getIR() {
        return false;
    }

    public default void setWristSpeed(double speed) {

    }
}