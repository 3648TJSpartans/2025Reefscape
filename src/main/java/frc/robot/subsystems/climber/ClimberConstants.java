package frc.robot.subsystems.climber;

public class ClimberConstants {

    public static final int leftMotorID = 12;
    public static final int rightMotorID = 13;

    public static final double kLeftP = 0;
    public static final double kLeftI = 0;
    public static final double kLeftD = 0;
    public static final double kLeftFF = 0;
    public static final double kLeftMinOutput = -1.0;
    public static final double kLeftMaxOutput = 1.0;

    public static final double leftOdometryFrequency = 100;
    public static final double kLeftMinRange = 0;
    public static final double kLeftMaxRange = 1;
    public static final boolean leftEncoderInverted = false;
    public static final double leftEncoderPositionFactor = 1;

    public static final double kRightP = 0;
    public static final double kRightI = 0;
    public static final double kRightD = 0;
    public static final double kRightFF = 0;
    public static final double kRightMinOutput = -1.0;
    public static final double kRightMaxOutput = 1.0;

    public static final double rightOdometryFrequency = 100;
    public static final double kRightMinRange = 0;
    public static final double kRightMaxRange = 1;
    public static final boolean rightEncoderInverted = false;
    public static final double rightEncoderPositionFactor = 1;

    public static final double climbLeftPosition = 0.138; // check
    public static final double climbRightPosition = 0; // change

}
