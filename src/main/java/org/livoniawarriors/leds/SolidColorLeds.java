package org.livoniawarriors.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class SolidColorLeds extends Command {
    ILedSubsystem leds;
    AddressableLEDBuffer ledBuffer;

    public SolidColorLeds(ILedSubsystem leds, Color color) {
        this.leds = leds;
        addRequirements(leds);
        ledBuffer = new AddressableLEDBuffer(leds.getLength());
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        leds.setData(ledBuffer);
    }

    @Override
    public void execute() {
        // since we set the color in init, no need to repeat it
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
