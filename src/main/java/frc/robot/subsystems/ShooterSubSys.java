// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubSys extends SubsystemBase {
    /** Creates a new Shooter. */

    // Run Shooter when operator presses A button

    // 2 Neo  motors - FR is positive to shoot. FL is negative to shoot.  
    // MIght need to invert shooter motors
    // Test for which direction positive/negative input causes?
    
    // once shooter wheels up to speed, run index motor to feed note into shooter
    // DRIVER right trigger for running INDEX motor

    private final CANSparkMax shooterMotorFR;
    private final CANSparkMax shooterMotorFL;
    private double shooterVelVoltsFR;
    private double shooterVelVoltsFL;
    private double shooterVelPctFR;
    private double shooterVelPctFL;


    public ShooterSubSys() {
        shooterMotorFR = new CANSparkMax(Constants.FR_SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
        shooterMotorFL = new CANSparkMax(Constants.FL_SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
        shooterMotorFR.setSmartCurrentLimit(Constants.FR_SHOOTER_MOTOR_SMART_CURRENT_LIMIT);
        shooterMotorFL.setSmartCurrentLimit(Constants.FL_SHOOTER_MOTOR_SMART_CURRENT_LIMIT);

        shooterVelPctFR = Constants.FR_SHOOTER_MOTOR_PCT;
        shooterVelPctFL = Constants.FL_SHOOTER_MOTOR_PCT;
        shooterVelVoltsFR = shooterVelPctFR * 12.0;
        shooterVelVoltsFL = shooterVelPctFL * 12.0;
    }

    public void runShooter() {
        shooterMotorFR.setVoltage(shooterVelVoltsFR);
        shooterMotorFL.setVoltage(shooterVelVoltsFL);
    }
    
    public void stopShooter() {
        shooterMotorFR.setVoltage(0);
        shooterMotorFL.setVoltage(0);
    }

    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
