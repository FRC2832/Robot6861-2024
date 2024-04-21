package org.livoniawarriors.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;

public class RainbowLeds extends Command {
    private ILedSubsystem leds;
    private int rumbleCounts;
    private int rainbowFirstPixelHue;
    private AddressableLEDBuffer ledBuffer;

    public RainbowLeds(ILedSubsystem leds) {
        this.leds = leds;
        addRequirements(leds);
        ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        // No op
    }

    @Override
    public void execute() {
        // For every pixel
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 60);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
        leds.setData(ledBuffer);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // No op
    }
}
