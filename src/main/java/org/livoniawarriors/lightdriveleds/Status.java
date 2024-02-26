package org.livoniawarriors.lightdriveleds;

public class Status {
    private byte raw;
    private Mode mode;

    public Status() {
        this.raw = 0;
        this.mode = Mode.NONE;
    }

    public byte getTripped() {
        return (byte) ((this.raw & 0xF0) >> 4);
    }

    public Boolean isEnabled() {
        if ((this.raw & 0x1) > 0) {
            return true;
        }
        return false;
    }

    public Mode getMode() {
        return this.mode;
    }

    public Byte getRaw() {
        return this.raw;
    }

    public void setRaw(final byte raw) {
        this.raw = raw;
    }

    public enum Mode {
        NONE("NONE", 0),
        IDLE("IDLE", 1),
        PWM("PWM", 2),
        CAN("CAN", 3),
        SERIAL("SERIAL", 4);

        private Mode(final String s, final int n) {
        }
    }
}