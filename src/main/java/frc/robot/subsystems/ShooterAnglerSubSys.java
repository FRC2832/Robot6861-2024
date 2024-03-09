// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class ShooterAnglerSubSys extends SubsystemBase {
    private final CANSparkMax linearActuatorMotor;
    private static final Timer TIMER = new Timer();
    private final double timerLim;
    private double linearActuatorVelVolts;
    private double linearActuatorVelPct;
    private double linearActuatorVelReversePct;
    private double linearActuatorVelVoltsReverse;

    /** Creates a new ShooterAnglerSubsys. */
    public ShooterAnglerSubSys() {
        // Linear Actuator Angle
        linearActuatorMotor = new CANSparkMax(Constants.LINEAR_ACTUATOR_MOTOR_CAN_ID, MotorType.kBrushed);
        linearActuatorMotor.setSmartCurrentLimit(Constants.LINEAR_ACTUATOR_MOTOR_SMART_CURRENT_LIMIT);

        linearActuatorVelPct = Constants.LINEAR_ACTUATOR_MOTOR_PCT / 100.0;
        linearActuatorVelReversePct = Constants.LINEAR_ACTUATOR_MOTOR_REVERSE_PCT / 100.0;
        linearActuatorVelVolts = linearActuatorVelPct * 12.0;
        linearActuatorVelVoltsReverse = linearActuatorVelReversePct * 12.0;

        timerLim = 1.75;
    }

    public void runLinearActuator() {
        TIMER.restart();
        while (TIMER.get() < timerLim) {
            linearActuatorMotor.setVoltage(linearActuatorVelVolts);
        }
        linearActuatorMotor.setVoltage(0.0);
        TIMER.stop();
        TIMER.reset();
    }

    public void runLinearActuatorReverse() {
        linearActuatorMotor.setVoltage(linearActuatorVelVoltsReverse);
    }

    public void stopLinearActuator() {
        linearActuatorMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
