package frc.robot.subsystems.sft;

public class SftConstants {
    public static final int sftMotorPin = 14;// TODO update placeholder
    public static final int irSensorPin = 4;// TODO update placeholder

    public static final double kP = .5;
    public static final double kI = 0;
    public static final double kD = .1;
    public static final double kFF = 0;

    public static final double kMinRange = -.3;
    public static final double kMaxRange = .3;

    public static double kSftOdometryFrequency = 100;
    public static final boolean encoderInverted = false;
    public static final double encoderPositionFactor = 1;
    public static final double kWristMinRange = -.5;
    public static final double kWristMaxRange = .5;

    public static final double kWristP = .3;
    public static final double kWristI = 0;
    public static final double kWristD = .1;
    public static final double kWristFF = 0;

}
