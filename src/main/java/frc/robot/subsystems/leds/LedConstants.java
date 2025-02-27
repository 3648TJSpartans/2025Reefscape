package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.autonCommands.CoralSequentialCmd;
import static edu.wpi.first.units.Units.*;

public final class LedConstants {

        // Values //
        public static int ledLength = 150;
        public static final int ledPWMID = 0;
        public static final int buffer1Start = 0;
        public static final int buffer1End = 37;
        public static final int buffer2Start = 38;
        public static final int buffer2End = 75;
        public static final int buffer3Start = 76;
        public static final int buffer3End = 111;
        public static final int buffer4Start = 112;
        public static final int buffer4End = 149;
        // Colors & Patterns //

        // Solid Colors
        public static LEDPattern green = LEDPattern.solid(Color.kGreen);
        public static LEDPattern red = LEDPattern.solid(Color.kRed);
        public static LEDPattern blue = LEDPattern.solid(Color.kBlue);
        public static LEDPattern teal = LEDPattern.solid(Color.kTeal);
        public static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
        public static LEDPattern purple = LEDPattern.solid(Color.kPurple);
        public static LEDPattern white = LEDPattern.solid(Color.kWhite);
        public static LEDPattern noColor = LEDPattern.solid(Color.kBlack);
        // Gradients
        public static LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        public static LEDPattern purpleGradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,
                        Color.kRed,
                        Color.kBlue);
        public static LEDPattern elevatorGradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous,
                        Color.kRed,
                        Color.kYellow, Color.kGreen, Color.kTeal); // Stoplight-esque gradient to use on the elevator

        // Animated Patterns
        public static LEDPattern breathingGreen = green.breathe(Seconds.of(2));
        public static LEDPattern breathingBlue = blue.breathe(Seconds.of(2));
        public static LEDPattern blinkingRed = red.blink(Seconds.of(2));
        public static LEDPattern blinkingBlue = blue.blink(Seconds.of(2));
        // Other Patterns
        public static LEDPattern elevatorMask = LEDPattern
                        .progressMaskLayer(() -> (double) CoralSequentialCmd.getLevel() / 4); // Just a mask, don't use
        public static LEDPattern elevatorHeight = elevatorGradient.mask(elevatorMask); // Use this
}