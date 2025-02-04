package frc.robot.subsystems.algae;

public class AlgaeConstants {
    public static final int liftMotorId = 14;
    public static final int intakeMotorId = 15;

    public static final double kLiftP = 0.1; // PID is not yet tuned
    public static final double kLiftI = 0;
    public static final double kLiftD = 0;
    public static final double kLiftFF = 0;
    public static final double kLiftMinRange = -1.0;
    public static final double kLiftMaxRange = 1.0;

    public static final boolean liftEncoderInverted = false;
    public static final double liftEncoderPositionFactor = 1.0;
    public static final double liftEncoderVelocityFactor = 1.0;
    public static final double liftOdometryFrequency = 100.0;

    public static final double liftIntakePosition = 0.806;
    public static final double liftUpWithBall = 0.619;
    public static final double liftUpWithoutBall = 0.58;
    public static final double shoot = 0.71;
}
