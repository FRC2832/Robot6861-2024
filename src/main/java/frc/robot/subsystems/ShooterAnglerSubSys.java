// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;

public class ShooterAnglerSubSys extends SubsystemBase {
    private final CANSparkMax linearActuatorMotor;
    private double linearActuatorVelVolts;
    private double linearActuatorVelPct;
    private double linearActuatorVelReversePct;
    private double linearActuatorVelVoltsReverse;

    /** Creates a new ShooterAnglerSubsys. */
    public ShooterAnglerSubSys() {
        // Linear Actuator Angle
        linearActuatorMotor = new CANSparkMax(Constants.LINEAR_ACTUATOR_MOTOR_CAN_ID, MotorType.kBrushed);
        linearActuatorMotor.setSmartCurrentLimit(Constants.LINEAR_ACTUATOR_MOTOR_SMART_CURRENT_LIMIT);

        linearActuatorVelPct = 1 / Constants.LINEAR_ACTUATOR_MOTOR_PCT;
        linearActuatorVelReversePct = 1 / Constants.LINEAR_ACTUATOR_MOTOR_REVERSE_PCT;
        linearActuatorVelVolts = linearActuatorVelPct * 12.0;
        linearActuatorVelVoltsReverse = linearActuatorVelReversePct * 12.0;
    }

    public void runLinearActuator() {
        linearActuatorMotor.setVoltage(linearActuatorVelVolts);
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
