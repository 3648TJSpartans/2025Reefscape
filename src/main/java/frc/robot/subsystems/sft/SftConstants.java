package frc.robot.subsystems.sft;

public class SftConstants {
    public static final int SftMotorPin = 14;// TODO update placeholder

    public static final double kP = 3;
    public static final double kI = 0;
    public static final double kD = .4;
    public static final double kFF = 0;

    public static final double kMinRange = -.2;
    public static final double kMaxRange = .2;

    public static final double endgameSetPoint = .32;
    public static final double dumpSetPoint = 0.18;

    public static double kSftOdometryFrequency = 100;
    public static final boolean encoderInverted = false;
    public static final double encoderPositionFactor = 1;

}
