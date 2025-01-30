package frc.robot.subsystems.coralSubsystems.Elevator;

public interface ElevatorIO {
    // allows us to move to a certain point
    public default void elevateTo(double setHeight) {
    }

    // allows us to stop the motor
    public default void stop() {
    }

    // allows us to know the heigh of the elevator in encoders unit
    public default double getHeight() {
        return 0;
    }

    // allows us to reset the encoder value
    public default void resetEncoder() {
    }

}
