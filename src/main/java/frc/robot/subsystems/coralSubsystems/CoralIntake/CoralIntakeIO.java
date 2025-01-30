package frc.robot.subsystems.coralSubsystems.CoralIntake;

public interface CoralIntakeIO {
    public default double getAngle() {
        return 0;
    }

    // allows us to rotate the arm of the coral subsystem to a given angle
    public default void RotateTo(double setangle) {
    }

    // allows us to stop the wrist motor
    public default void stopWristMotor() {
    }

    // allows us to stop the intake motor
    public default void stopIntakeMotor() {
    }

    // allows us to take in a coral
    public default void takeIN() {
    }

    // allows us to take out a coral
    public default void takeOUT() {
    }
}