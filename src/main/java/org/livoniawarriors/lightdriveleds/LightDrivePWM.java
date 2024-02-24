package org.livoniawarriors.lightdriveleds;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;

public final class LightDrivePWM {
    private DigitalOutput pwmSelect;
    // private DigitalOutput m_pwm_value;
    private Servo servoBank1;
    private Servo servoBank2;
    private int[] matrix;
    private boolean typeServo;

    public LightDrivePWM(final Servo bank1, final Servo bank2) {
        this.servoBank1 = bank1;
        this.servoBank2 = bank2;
        this.matrix = new int[12];
        this.typeServo = true;
    }

    public void update() {
        double newduty = 0.0;
        int channels = ((this.matrix[0] > 127) ? 16 : 0) | ((this.matrix[1] > 127) ? 32 : 0)
                | ((this.matrix[2] > 127) ? 64 : 0) | ((this.matrix[3] > 127) ? 128 : 0)
                | ((this.matrix[4] > 127) ? 256 : 0) | ((this.matrix[5] > 127) ? 512 : 0);
        newduty = channels / 1023.0;
        if (this.typeServo) {
            this.servoBank1.set(newduty);
        } else {
            this.pwmSelect.updateDutyCycle(newduty);
        }
        channels = (((this.matrix[6] > 127) ? 16 : 0) | ((this.matrix[7] > 127) ? 32 : 0)
                | ((this.matrix[8] > 127) ? 64 : 0) | ((this.matrix[9] > 127) ? 128 : 0)
                | ((this.matrix[10] > 127) ? 256 : 0) | ((this.matrix[11] > 127) ? 512 : 0));
        newduty = channels / 1023.0;
        if (this.typeServo) {
            this.servoBank2.set(newduty);
        } else {
            this.pwmSelect.updateDutyCycle(newduty);
        }
    }

    public void setColor(int ch, final Color color) {
        if (ch < 1 || ch > 4) {
            return;
        }
        ch = --ch * 3;
        this.matrix[ch] = (int) (color.green * 255);
        this.matrix[ch + 1] = (int) (color.red * 255);
        this.matrix[ch + 2] = (int) (color.blue * 255);
    }

    public void setLevel(final int ch, final int level) {
        if (ch < 1 || ch > 12 || level < 0 || level > 255) {
            return;
        }
        this.matrix[ch] = level;
    }
}