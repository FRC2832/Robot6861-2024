// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
<<<<<<< Updated upstream

import frc.robot.Constants;
=======
>>>>>>> Stashed changes

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        climberEncoder = climberMotor.getEncoder(); // TODO: Correct countsPerRev
        climberMotor.setSmartCurrentLimit(Constants.CLIMBER_MOTOR_SMART_CURRENT_LIMIT);
        upClimbVelPct = Constants.UPCLIMB_MOTOR_PCT;
        downClimbVelPct = Constants.DOWNCLIMB_MOTOR_PCT;
        upClimbVelVolts = upClimbVelPct * 12.0;
        downClimbVelVolts = downClimbVelPct * 12.0;
    }

    public void resetEncoders() {
        climberEncoder.setPosition(0.0);
    }

     // Runs the Climb Motor in a positive direction to raise climber arm up.  //TODO:  check if this is correct

    public void runClimberUp() {
        SmartDashboard.putNumber("Climb motor encoder - up",climberEncoder.getPosition());
        climberMotor.setVoltage(upClimbVelVolts);
    }

    // Runs the Climber eMotor in a negative direction
    public void runClimberDown() {
        SmartDashboard.putNumber("Climb motor encoder - down",climberEncoder.getPosition());
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
