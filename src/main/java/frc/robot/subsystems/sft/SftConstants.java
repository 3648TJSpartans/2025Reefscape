package frc.robot.subsystems.sft;

public class SftConstants {
    public static final int sftMotorPin = 14;// TODO update placeholder
    public static final int irSensorPin = 4;// TODO update placeholder

    public static final double kP = 3;
    public static final double kI = 0;
    public static final double kD = .1;
    public static final double kFF = 0;

    public static final double kMinRange = -.5;
    public static final double kMaxRange = .5;

    public static final double endgameSetPoint = .32;

    public static double kSftOdometryFrequency = 100;
    public static final boolean encoderInverted = false;
    public static final double encoderPositionFactor = 1;

}
