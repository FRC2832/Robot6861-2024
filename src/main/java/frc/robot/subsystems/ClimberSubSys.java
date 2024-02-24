// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubSys extends SubsystemBase {
    /** Creates a new Climber. */
     // if Operator DPad up is climber up.  run climber motor CAN Id 5
     // TODO: double check CAN ID on robot, and update here

    // Neo?  motor
    // Climb up is in positive volts?????  CHECK
    // CLIMber Down is negative volts????  CHECK
    // 

    private final CANSparkMax climberMotor;
    private final RelativeEncoder climberEncoder;
    private double upClimbVelVolts;
    private double downClimbVelVolts;
    private double upClimbVelPct;
    private double downClimbVelPct;



    public ClimberSubSys() {
        climberMotor = new CANSparkMax(Constants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climberMotor.setSmartCurrentLimit(Constants.CLIMBER_MOTOR_SMART_CURRENT_LIMIT);
        climberEncoder = climberMotor.getEncoder();
        upClimbVelPct = Constants.UPCLIMB_MOTOR_PCT;
        downClimbVelPct = Constants.DOWNCLIMB_MOTOR_PCT;
        upClimbVelVolts = upClimbVelPct * 12.0;
        downClimbVelVolts = downClimbVelPct * 12.0;
    }

     // Runs the Climb Motor in a positive direction to raise climber arm up.  //TODO:  check if this is correct

    public void runClimberUp() {
        climberMotor.setVoltage(upClimbVelVolts);
    }

    // Runs the Climber eMotor in a negative direction
    public void runClimberDown() {
        climberMotor.setVoltage(downClimbVelVolts);
    }

    public void stopClimberMotor() {
        climberMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
