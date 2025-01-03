// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpScorerSubSys extends SubsystemBase {
  private final CANSparkMax ampMotor;
  private final RelativeEncoder ampEncoder;
  

  //private double ampMotorVelPct;
  //private double ampMotorReverseVelPct;
  //private double ampMotorVolts;
  //private double ampMotorReverseVolts;

  private SparkPIDController ampPIDController;
  private double kP;
  private double kI;
  private double kD;
  private double kIz;
  private double kFF;
  private double kMaxOutput;
  private double kMinOutput;

  /** Creates a new AmpScorerSubSys. */
  public AmpScorerSubSys() {
    ampMotor = new CANSparkMax(Constants.AMP_MOTOR_CAN_ID, MotorType.kBrushless);
    ampMotor.setSmartCurrentLimit(Constants.AMP_MOTOR_SMART_CURRENT_LIMIT);
    ampMotor.setSecondaryCurrentLimit(Constants.AMP_MOTOR_SECONDARY_CURRENT_LIMIT);

    //from https://www.revrobotics.com/development-spark-max-users-manual/#section-3-3-2-1
    ampMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);  //to help reduce CANbus high utilization
    ampMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 110);  // TODO: might be able to go higher than 100....
    ampMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
    ampMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 150);  
    ampMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500);
    ampMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    ampMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);


    ampEncoder = ampMotor.getEncoder();
    ampEncoder.setPosition(0.0);
    ampPIDController = ampMotor.getPIDController();


    //ampMotorVelPct = Constants.AMP_UP_PCT / 100.0; // TODO: Possibly switch with AMP_DOWN_PCT
    //ampMotorReverseVelPct = Constants.AMP_DOWN_PCT / 100.0;
    //ampMotorVolts = ampMotorVelPct * 12.0;
    //ampMotorReverseVolts = ampMotorReverseVelPct * 12.0;

    ampMotor.setIdleMode(IdleMode.kCoast); // set to coast when needing to work on Amp, set to brake for comps and practice
  }

  public void resetEncoders() {
    ampEncoder.setPosition(0.0);
}

  public void runAmpMotor() {  //runs amp arm up
     
    // PID coefficients
      kP = 1.4;  //was 1.0 
      //kI = 0.0;
      //kD = 0.0;
     // kIz = 0.0;
     // kFF = 0.00;
      kMaxOutput = 0.99;
      kMinOutput = -0.98;

      // set PID coefficients
      ampPIDController.setP(kP);
      //ampPIDController.setI(kI);
     // ampPIDController.setD(kD);
     // ampPIDController.setIZone(kIz);
     // ampPIDController.setFF(kFF);
      ampPIDController.setOutputRange(kMinOutput, kMaxOutput);

      double rotations = 287.0;   //was 288, reduced so can have some overshoot in exchange for motor getting up to speed quicker.

      ampPIDController.setReference(rotations, CANSparkBase.ControlType.kPosition);
        //ampMotor.setVoltage(ampMotorVolts);

      SmartDashboard.putNumber("SetPoint Amp", rotations);
      SmartDashboard.putNumber("ProcessVariable Amp", ampEncoder.getPosition());
      SmartDashboard.putNumber("Motor Speed Amp", ampEncoder.getVelocity());
    
  }

  public void runAmpMotorReverse() {  // runs amp arm down
    // PID coefficients
      kP = 0.3;
     // kI = 0.0;
     // kD = 0.0;
     // kIz = 0.0;
     // kFF = 0.0;
      kMaxOutput = 0.98;
      kMinOutput = -0.98;

    // set PID coefficients
      ampPIDController.setP(kP);
      //ampPIDController.setI(kI);
      //ampPIDController.setD(kD);
     // ampPIDController.setIZone(kIz);
     // ampPIDController.setFF(kFF);
      ampPIDController.setOutputRange(kMinOutput, kMaxOutput);

      double rotations = 0.0;  

      //if rotations < 
      ampPIDController.setReference(rotations, CANSparkBase.ControlType.kPosition);
      //ampMotor.setVoltage(ampMotorReverseVolts);

      // Uncomment these for development, testing or debugging work:
      //SmartDashboard.putNumber("SetPoint Amp", rotations);
      //SmartDashboard.putNumber("ProcessVariable Amp", ampEncoder.getPosition());
      //SmartDashboard.putNumber("Motor Speed Amp", ampEncoder.getVelocity());

      //if (ampEncoder.getVelocity() < 10.0 );
          //stopAmpMotor();

      
  }

  public void stopAmpMotor() {
    ampMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Uncomment this for development, testing or debugging work:
    // SmartDashboard.putNumber("Amp Encoder Position", ampEncoder.getPosition());
  }
}
