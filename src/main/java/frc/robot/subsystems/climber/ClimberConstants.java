package frc.robot.subsystems.climber;

public class ClimberConstants {

    public static final int leadMotorID = 12; // left
    public static final int followMotorID = 13; // right

    public static final double kLeadP = 0;
    public static final double kLeadI = 0;
    public static final double kLeadD = 0;
    public static final double kLeadFF = 0;
    public static final double kLeadMinOutput = -0.3;
    public static final double kLeadMaxOutput = 0.3;

    public static final double leadOdometryFrequency = 100;
    public static final double kLeadMinRange = 0;
    public static final double kLeadMaxRange = 1;
    public static final boolean leadEncoderInverted = false;
    public static final double leadEncoderPositionFactor = 1;

    public static final double kFollowP = 0;
    public static final double kFollowI = 0;
    public static final double kFollowD = 0;
    public static final double kFollowFF = 0;
    public static final double kFollowMinOutput = -.3;
    public static final double kFollowMaxOutput = .3;

    public static final double followOdometryFrequency = 100;
    public static final double kFollowMinRange = 0;
    public static final double kFollowMaxRange = 1;
    public static final boolean followEncoderInverted = false;
    public static final double followEncoderPositionFactor = 1;

    public static final double upPosition1 = 0.025;
    public static final double upPosition2 = 0.020;

    public static final double downPosition1 = 1;
    public static final double downPosition2 = 0.350; // may need testing
    // These might need flipping

}
