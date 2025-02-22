package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.autonCommands.CoralSequentialCmd;

public final class LedConstants {
    public static final int ledLength = 152;
    public static final int ledPWMID = 1;
    public static final int topBarLedStart = 0;
    public static final int topBarLedStop = 52;

    // Colors
    public static LEDPattern green = LEDPattern.solid(Color.kGreen);
    public static LEDPattern red = LEDPattern.solid(Color.kRed);
    public static LEDPattern blue = LEDPattern.solid(Color.kBlue);
    public static LEDPattern teal = LEDPattern.solid(Color.kTeal);
    public static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    public static LEDPattern purple = LEDPattern.solid(Color.kPurple);
    public static LEDPattern white = LEDPattern.solid(Color.kWhite);

    public static LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    public static LEDPattern purpleGradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
    
    // Other patterns
    public static LEDPattern elevatorMask = LEDPattern.progressMaskLayer(() -> CoralSequentialCmd.getLevel() / 4);
    public static LEDPattern elevatorHeight = purple.mask(elevatorMask);
}