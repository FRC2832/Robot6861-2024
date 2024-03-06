// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubSys extends SubsystemBase {
    /** Creates a new Intake. */
    // if Operator right trigger held, run intake motors CAN Id 1 & 2

    // Neo 550 motors
    // Intake in is positive volts
    // eject is negative volts
    // once note is fully in indexer, run intake backwards

    private final CANSparkMax intakeMotor;
    private double intakeVelVolts;
    private double outtakeVelVolts;
    private double intakeVelPct;
    private double outtakeVelPct;

    public IntakeSubSys() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(Constants.INTAKE_MOTOR_SMART_CURRENT_LIMIT);
        intakeMotor.setSecondaryCurrentLimit(Constants.INTAKE_MOTOR_SECONDARY_CURRENT_LIMIT);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeVelPct = Constants.INTAKE_MOTOR_PCT;
        outtakeVelPct = Constants.OUTTAKE_MOTOR_PCT;
        intakeVelVolts = intakeVelPct * 12.0;
        outtakeVelVolts = outtakeVelPct * 12.0;
    }

    // Runs the IntakeMotors in a positive direction(Inwards)
    public void runIntake() {
        intakeMotor.setVoltage(intakeVelVolts);
    }

    // Runs the IntakeMotors in a negative direction(Outwards)
    public void runOuttake() {
        intakeMotor.setVoltage(outtakeVelVolts);
    }

    public void stopIntakeMotors() {
        intakeMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
