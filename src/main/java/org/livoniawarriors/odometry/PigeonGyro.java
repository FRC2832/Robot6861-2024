package org.livoniawarriors.odometry;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class PigeonGyro implements IGyroHardware {
    private PigeonIMU pigeon;
    private double[] yprDeg;
    private double[] xyzMps;

    public PigeonGyro(int id) {
        pigeon = new PigeonIMU(id);
        yprDeg = new double[3];
        xyzMps = new double[3];
    }

    public PigeonGyro(TalonSRX motor) {
        pigeon = new PigeonIMU(motor);
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
        return yprDeg[0];  //was negative when Pigeon was upside down.
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
