package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;

public class LedSubsystem extends SubsystemBase {

    private final AddressableLEDBuffer ledBuffer;

    public final AddressableLEDBufferView elevatorBuffer;
    public final AddressableLEDBufferView leftGuideBuffer;
    public final AddressableLEDBufferView rightGuideBuffer;

    private final AddressableLED led;

    public LedSubsystem() {
        ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
        led = new AddressableLED(LedConstants.ledPWMID);

        // Create sub-buffers
        elevatorBuffer = ledBuffer.createView(LedConstants.elevatorLedStart, LedConstants.elevatorLedEnd);
        leftGuideBuffer = ledBuffer.createView(LedConstants.leftGuideStart, LedConstants.leftGuideEnd);
        rightGuideBuffer = ledBuffer.createView(LedConstants.rightGuideStart, LedConstants.rightGuideEnd);
        
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

    }

    //Writes a specified LED pattern to the buffer. A bunch of patterns can be found in LEDConstants, or you can make your own.
    public void setLedPattern(LEDPattern pattern, AddressableLEDBufferView subBuffer) {
        pattern.applyTo(subBuffer);
        led.setData(ledBuffer);
    }
}