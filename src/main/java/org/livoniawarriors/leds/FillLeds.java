package org.livoniawarriors.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class FillLeds extends Command {
    private static final double STEP_TIME = 0.04; // increment every 40ms
    private ILedSubsystem leds;
    private AddressableLEDBuffer ledBuffer;
    private double startTime;
    private int currentLight;
    private Color color;

    public FillLeds(ILedSubsystem leds, Color color) {
        this.leds = leds;
        this.color = color;
        addRequirements(leds);
        ledBuffer = new AddressableLEDBuffer(leds.getLength());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // calculate the current light
        double time = Timer.getFPGATimestamp() - startTime;
        currentLight = (int) (time / STEP_TIME);

        // set the pattern
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i > currentLight) {
                color = Color.kBlack;
            }
            ledBuffer.setLED(i, color);
        }
        leds.setData(ledBuffer);
    }

    @Override
    public boolean isFinished() {
        return currentLight > ledBuffer.getLength();
    }

    @Override
    public void end(boolean interrupted) {
        // No op
    }
}
