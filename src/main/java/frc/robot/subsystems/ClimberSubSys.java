// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkPIDController;

public class ClimberSubSys extends SubsystemBase {
    /** Creates a new Climber. */
    // if Operator DPad up is climber up. run climber motor CAN Id 5
    // TODO: double check CAN ID on robot, and update here

    // Neo? motor
    // Climb up is in positive volts????? CHECK
    // CLIMber Down is negative volts???? CHECK
    //

    private final CANSparkMax climberMotor;
    private final RelativeEncoder climberEncoder;
    private double upClimbVelVolts;
    private double downClimbVelVolts;
    private double upClimbVelPct;
    private double downClimbVelPct;
    // private PIDController climberPID;
    private SparkPIDController climberPIDController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    public ClimberSubSys() {
        climberMotor = new CANSparkMax(Constants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder(); // TODO: Correct countsPerRev
        climberEncoder.setPosition(0.0);
        climberPIDController = climberMotor.getPIDController();
        climberMotor.setSmartCurrentLimit(Constants.CLIMBER_MOTOR_SMART_CURRENT_LIMIT);
        climberMotor.setSecondaryCurrentLimit(Constants.CLIMBER_MOTOR_SECONDARY_CURRENT_LIMIT);
        upClimbVelPct = Constants.UPCLIMB_MOTOR_PCT;
        downClimbVelPct = Constants.DOWNCLIMB_MOTOR_PCT;
        upClimbVelVolts = upClimbVelPct * 12.0;
        downClimbVelVolts = downClimbVelPct * 12.0;

        climberMotor.setIdleMode(IdleMode.kBrake);  //set to coast when needing to work on climber

        
    }

    public void resetEncoders() {
        climberEncoder.setPosition(0.0);
    }

    public double showEncoders() {
        //climberEncoder.getPosition();
        return climberEncoder.getPosition();
    }





    // Runs the Climb Motor in a positive direction to raise climber arm up. //TODO:
    // check if this is correct

    public void runClimberUp() {
        SmartDashboard.putNumber("Climb motor encoder - up", climberEncoder.getPosition());

        // PID coefficients
        kP = 0.15;
        kI = 0.0;
        kD = 0.0;
        kIz = 0.0;
        kFF = 0.0;
        kMaxOutput = 0.5;
        kMinOutput = -0.9;

        // set PID coefficients
        climberPIDController.setP(kP);
        climberPIDController.setI(kI);
        climberPIDController.setD(kD);
        climberPIDController.setIZone(kIz);
        climberPIDController.setFF(kFF);
        climberPIDController.setOutputRange(kMinOutput, kMaxOutput);
        
         double rotations = 50.0;
 
         //climberMotor.setVoltage(downClimbVelVolts);
 
         climberPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
     
         SmartDashboard.putNumber("SetPoint", rotations);
         SmartDashboard.putNumber("ProcessVariable", climberEncoder.getPosition());

        //climberMotor.setVoltage(upClimbVelVolts);
    }

    // Runs the Climber eMotor in a negative direction
    public void runClimberDown() {
        SmartDashboard.putNumber("Climb motor encoder - down", climberEncoder.getPosition());

         // PID coefficients
         kP = 0.2;
         kI = 0.0;
         kD = 0.0;
         kIz = 0.0;
         kFF = 1.0;
         kMaxOutput = 0.9;
         kMinOutput = -0.9;
 
         // set PID coefficients
         climberPIDController.setP(kP);
         climberPIDController.setI(kI);
         climberPIDController.setD(kD);
         climberPIDController.setIZone(kIz);
         climberPIDController.setFF(kFF);
         climberPIDController.setOutputRange(kMinOutput, kMaxOutput);
       
        double rotations = -8.0;
        

        //climberMotor.setVoltage(downClimbVelVolts);

        climberPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", climberEncoder.getPosition());
       
    }

    public void stopClimberMotor() {
        climberMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climb motor encoder", climberEncoder.getPosition());
    }
}
