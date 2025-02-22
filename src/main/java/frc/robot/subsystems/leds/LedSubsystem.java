package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;

public class LedSubsystem extends SubsystemBase {

    private final AddressableLEDBuffer ledBuffer;

    private final AddressableLED led;

    public LedSubsystem() {
        ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
        led = new AddressableLED(LedConstants.ledPWMID);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

    }

    //Writes a specified LED pattern to the buffer. A bunch of patterns can be found in LEDConstants, or you can make your own.
    public void setLedPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}