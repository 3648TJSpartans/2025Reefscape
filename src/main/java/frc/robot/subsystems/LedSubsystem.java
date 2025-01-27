package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

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

    public void setColor(int rgb[], int startValue, int endValue) {

        for (int i = startValue; i < endValue; i++) {
            ledBuffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
        led.setData(ledBuffer);
    }

    public void setIntakeColor(DigitalInput irSensor) {
        if (!irSensor.get()) {
            setColor(LedConstants.yesNoteRGB, LedConstants.topBarLedStart, LedConstants.topBarLedStop);
        } else {
            setColor(LedConstants.noNoteRGB, LedConstants.topBarLedStart, LedConstants.topBarLedStop);
        }
    }
} 
