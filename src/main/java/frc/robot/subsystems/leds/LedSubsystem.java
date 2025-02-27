package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;

public class LedSubsystem extends SubsystemBase {

    public final AddressableLEDBuffer ledBuffer;

    public final AddressableLEDBufferView buffer1;
    public final AddressableLEDBufferView buffer2;
    public final AddressableLEDBufferView buffer3;
    public final AddressableLEDBufferView buffer4;

    private final AddressableLED led;

    public LedSubsystem() {
        ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);
        led = new AddressableLED(LedConstants.ledPWMID);

        // Create sub-buffers
        buffer1 = ledBuffer.createView(LedConstants.buffer1Start, LedConstants.buffer1End);
        buffer2 = ledBuffer.createView(LedConstants.buffer2Start, LedConstants.buffer2End);
        buffer3 = ledBuffer.createView(LedConstants.buffer3Start, LedConstants.buffer3End);
        buffer4 = ledBuffer.createView(LedConstants.buffer4Start, LedConstants.buffer4End);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

    }

    // Writes a specified LED pattern to the buffer. A bunch of patterns can be
    // found in LEDConstants, or you can make your own.
    public void setLedPattern(LEDPattern pattern, AddressableLEDBufferView subBuffer) {
        pattern.applyTo(subBuffer);
        led.setData(ledBuffer);
    }

    public void turnLedsOff() {

        LedConstants.noColor.applyTo(ledBuffer);
        led.setData(ledBuffer);

    }

    // left or right
    public void setGlobalPattern(LEDPattern pattern, int level) {
        if (level == 1) {
            pattern.applyTo(buffer1);
            LedConstants.noColor.applyTo(buffer2);
            LedConstants.noColor.applyTo(buffer3);
            LedConstants.noColor.applyTo(buffer4);

        }
        if (level == 2) {
            pattern.applyTo(buffer1);
            pattern.applyTo(buffer2);
            LedConstants.noColor.applyTo(buffer3);
            LedConstants.noColor.applyTo(buffer4);

        }
        if (level == 3) {
            pattern.applyTo(buffer1);
            pattern.applyTo(buffer2);
            pattern.applyTo(buffer3);
            LedConstants.noColor.applyTo(buffer4);

        }
        if (level == 4) {
            pattern.applyTo(buffer1);
            pattern.applyTo(buffer2);
            pattern.applyTo(buffer3);
            pattern.applyTo(buffer4);

        }
        led.setData(ledBuffer);
    }
}