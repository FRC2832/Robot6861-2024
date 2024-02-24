package org.livoniawarriors.lightdriveleds;

import java.nio.*;
import edu.wpi.first.wpilibj.util.*;
import edu.wpi.first.hal.can.*;
import edu.wpi.first.hal.util.UncleanStatusException;

public final class LightDriveCAN {
    private static int LD_ADDR;
    private ByteBuffer m_matrix;
    private RxPacket m_rx;
    // private boolean m_init;
    private byte[] rxdata;
    private static ByteBuffer timestamp;
    private static ByteBuffer rxid;

    static {
        LightDriveCAN.LD_ADDR = 33882112;
    }

    public LightDriveCAN() {
        this.m_matrix = ByteBuffer.allocate(16);
        // this.m_init = false;
        this.m_rx = new RxPacket();
        (LightDriveCAN.timestamp = ByteBuffer.allocateDirect(4)).order(ByteOrder.LITTLE_ENDIAN);
        (LightDriveCAN.rxid = ByteBuffer.allocateDirect(4)).order(ByteOrder.LITTLE_ENDIAN);
    }

    public LightDriveCAN(final int addr) {
    }

    public void update() {
        final byte[] txdata = new byte[8];
        LightDriveCAN.rxid.putInt(LightDriveCAN.LD_ADDR + 4);
        LightDriveCAN.rxid.rewind();
        try {
            this.m_matrix.get(txdata, 0, 8);
            CANJNI.FRCNetCommCANSessionMuxSendMessage(LightDriveCAN.LD_ADDR, txdata, 100);
            this.m_matrix.get(txdata, 0, 8);
            CANJNI.FRCNetCommCANSessionMuxSendMessage(LightDriveCAN.LD_ADDR + 1, txdata, 100);
        } catch (UncleanStatusException ex) {
        }
        this.m_matrix.rewind();
        try {
            this.rxdata = CANJNI.FRCNetCommCANSessionMuxReceiveMessage(LightDriveCAN.rxid.asIntBuffer(), 536870911,
                    LightDriveCAN.timestamp);
            if (this.rxdata.length > 7) {
                this.m_rx.setBytes(this.rxdata);
            }
        } catch (CANMessageNotFoundException ex2) {
        }
    }

    public float getCurrent(final int ch) {
        float current = 0.0f;
        switch (ch) {
            case 1: {
                current = this.m_rx.i1;
                break;
            }
            case 2: {
                current = this.m_rx.i2;
                break;
            }
            case 3: {
                current = this.m_rx.i3;
                break;
            }
            case 4: {
                current = this.m_rx.i4;
                break;
            }
            default: {
                current = -10.0f;
                break;
            }
        }
        return current / 10.0f;
    }

    public float getTotalCurrent() {
        return (this.m_rx.i1 + this.m_rx.i2 + this.m_rx.i3 + this.m_rx.i4) / 10.0f;
    }

    public float getVoltage() {
        return this.m_rx.vin / 10.0f;
    }

    public int getFWVersion() {
        return this.m_rx.fw;
    }

    public Status getStatus() {
        return this.m_rx.status;
    }

    public int getPWMs(final int ch) {
        if (ch > 2 || ch < 1) {
            return -1;
        }
        return (ch > 1) ? (this.m_rx.pwmVals >> 8) : (this.m_rx.pwmVals & 0xFF);
    }

    public void setColor(int ch, final Color color) {
        if (ch < 1 || ch > 4) {
            return;
        }
        ch = --ch * 3;
        this.m_matrix.array()[ch] = (byte) color.green;
        this.m_matrix.array()[ch + 1] = (byte) color.red;
        this.m_matrix.array()[ch + 2] = (byte) color.blue;
    }

    public void setColor(int ch, final Color color, final double brightness) {
        if (ch < 1 || ch > 4) {
            return;
        }
        byte red = (byte) (color.red * brightness);
        byte green = (byte) (color.green * brightness);
        byte blue = (byte) (color.blue * brightness);
        ch = --ch * 3;
        this.m_matrix.array()[ch] = (byte) green;
        this.m_matrix.array()[ch + 1] = (byte) red;
        this.m_matrix.array()[ch + 2] = (byte) blue;
    }

    public void setLevel(final int ch, final byte level) {
        if (ch < 1 || ch > 12 || level < 0 || level > 255) {
            return;
        }
        this.m_matrix.array()[ch] = level;
    }
}