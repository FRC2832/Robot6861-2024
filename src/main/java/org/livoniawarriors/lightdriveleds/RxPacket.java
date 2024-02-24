package org.livoniawarriors.lightdriveleds;

final class RxPacket {
    byte i1;
    byte i2;
    byte i3;
    byte i4;
    byte vin;
    Status status;
    byte pwmVals;
    byte fw;

    RxPacket() {
        this.status = new Status();
        this.i1 = 0;
        this.i2 = 0;
        this.i3 = 0;
        this.i4 = 0;
        this.vin = 0;
        this.pwmVals = 0;
        this.fw = 0;
    }

    byte[] getBytes() {
        final byte[] tempdata = { this.i1, this.i2, this.i3, this.i4, this.vin, this.status.getRaw(), this.pwmVals,
                this.fw };
        return tempdata;
    }

    void setBytes(final byte[] data) {
        this.i1 = data[0];
        this.i2 = data[1];
        this.i3 = data[2];
        this.i4 = data[3];
        this.vin = data[4];
        this.status.setRaw(data[5]);
        this.pwmVals = data[6];
        this.fw = data[7];
    }
}