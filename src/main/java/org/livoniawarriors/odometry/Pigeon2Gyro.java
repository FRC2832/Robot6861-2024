package org.livoniawarriors.odometry;

import com.ctre.phoenix.sensors.Pigeon2;

@SuppressWarnings("removal")
public class Pigeon2Gyro implements IGyroHardware {
    private Pigeon2 pigeon;
    private double[] yprDeg;
    private double[] xyzMps;

    public Pigeon2Gyro(int id) {
        this(id, null);
    }

    public Pigeon2Gyro(int id, String busName) {
        if (busName != null) {
            pigeon = new Pigeon2(id, busName);
        } else {
            pigeon = new Pigeon2(id);
        }
        yprDeg = new double[3];
        xyzMps = new double[3];
    }

    @Override
    public void updateHardware() {
        short[] temp = new short[3];
        pigeon.getYawPitchRoll(yprDeg);
        pigeon.getBiasedAccelerometer(temp);

        for (int i = 0; i < 3; i++) {
            xyzMps[i] = ((double) temp[i]) / 16384;
        }
    }

    @Override
    public double getGyroAngle() {
        return yprDeg[0];
    }

    @Override
    public double zeroGyroAngle() {  // same as getGYroANgle for now.  WE don't use pigeon2
        return yprDeg[0];
    }

    @Override
    public double getPitchAngle() {
        return yprDeg[1];
    }

    @Override
    public double getRollAngle() {
        return yprDeg[2];
    }

    @Override
    public double getXAccel() {
        return xyzMps[0];
    }

    @Override
    public double getYAccel() {
        return xyzMps[1];
    }

    @Override
    public double getZAccel() {
        return xyzMps[2];
    }
}
