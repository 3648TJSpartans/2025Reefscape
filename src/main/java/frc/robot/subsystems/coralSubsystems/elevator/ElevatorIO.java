package frc.robot.subsystems.coralSubsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    // allows us to move to a certain point
    public default void elevateTo(double setHeight) {
    }

    public default void updateValues() {
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

    public default void setSpeed(double speed) {
    }

    public default void updateLimitSwitch() {
    }

    public default boolean atBottom() {
        return true;
    }

    public default void zeroElevator() {
    }
}
