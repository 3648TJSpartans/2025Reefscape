package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.autonCommands.CoralSequentialCmd;

public final class LedConstants {

    // Values //
    public static final int ledLength = 150;
    public static final int ledPWMID = 1;
    public static final int elevatorLedStart = 0;
    public static final int elevatorLedEnd = 60;

    // Colors & Patterns //
    
    // Solid Colors
    public static LEDPattern green = LEDPattern.solid(Color.kGreen);
    public static LEDPattern red = LEDPattern.solid(Color.kRed);
    public static LEDPattern blue = LEDPattern.solid(Color.kBlue);
    public static LEDPattern teal = LEDPattern.solid(Color.kTeal);
    public static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    public static LEDPattern purple = LEDPattern.solid(Color.kPurple);
    public static LEDPattern white = LEDPattern.solid(Color.kWhite);

    // Gradients
    public static LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    public static LEDPattern purpleGradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
    
    // Other Patterns
    public static LEDPattern elevatorMask = LEDPattern.progressMaskLayer(() -> CoralSequentialCmd.getLevel() / 4); // Just a mask, don't use
    public static LEDPattern elevatorHeight = purple.mask(elevatorMask); // Use this
}