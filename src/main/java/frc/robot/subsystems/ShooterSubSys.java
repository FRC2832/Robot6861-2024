// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.livoniawarriors.Logger;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ShooterSubSys extends SubsystemBase {
    /** Creates a new Shooter. */

    // Run Shooter when operator presses A button

    // 2 Neo motors - FR is positive to shoot. FL is negative to shoot.
    // MIght need to invert shooter motors
    // Test for which direction positive/negative input causes?

    // once shooter wheels up to speed, run index motor to feed note into shooter
    // DRIVER right trigger for running INDEX motor

    private final CANSparkMax shooterMotorFR;
    private final CANSparkMax shooterMotorFL;
    private final RelativeEncoder shooterMotorFREncoder;
    private final RelativeEncoder shooterMotorFLEncoder;
    private double shooterVelVoltsFR;
    private double shooterVelVoltsFL;
    private double shooterVelPctFR;
    private double shooterVelPctFL;
    private double shooterVelRPM;
    private SparkPIDController shooterPIDControllerFR;
    private SparkPIDController shooterPIDControllerFL;
    private double kP;
    private double kI;
    private double kD;
    private double kIz;
    private double kFF;
    private double kMaxOutputFL;
    private double kMinOutputFL;
    private double kMaxOutputFR;
    private double kMinOutputFR;
    private double maxRPM;



    public ShooterSubSys() {
        // Main Shooter Module
        shooterMotorFR = new CANSparkMax(Constants.FR_SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
        shooterMotorFL = new CANSparkMax(Constants.FL_SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
        shooterMotorFREncoder = shooterMotorFR.getEncoder();
        shooterMotorFLEncoder = shooterMotorFL.getEncoder();
        shooterMotorFREncoder.setPosition(0.0);
        shooterMotorFLEncoder.setPosition(0.0);
        shooterPIDControllerFR = shooterMotorFR.getPIDController();
        shooterPIDControllerFL = shooterMotorFL.getPIDController();
        shooterMotorFR.setSmartCurrentLimit(Constants.FR_SHOOTER_MOTOR_SMART_CURRENT_LIMIT);
        shooterMotorFL.setSmartCurrentLimit(Constants.FL_SHOOTER_MOTOR_SMART_CURRENT_LIMIT);

        shooterVelPctFR = Constants.FR_SHOOTER_MOTOR_PCT_SPEAKER / 100.0;
        shooterVelPctFL = Constants.FL_SHOOTER_MOTOR_PCT_SPEAKER / 100.0;
        shooterVelVoltsFR = shooterVelPctFR * 12.0;
        shooterVelVoltsFL = shooterVelPctFL * 12.0;

        shooterMotorFR.setIdleMode(IdleMode.kBrake); 
        shooterMotorFL.setIdleMode(IdleMode.kBrake);
    }

    public void resetEncoders() {
        shooterMotorFLEncoder.setPosition(0.0);
        shooterMotorFREncoder.setPosition(0.0);
    }


    public void runShooterLowSpeed() {
        shooterMotorFL.set(Constants.FL_SHOOTER_MOTOR_PCT_AMP / 100.0);
        shooterMotorFR.set(Constants.FR_SHOOTER_MOTOR_PCT_AMP / 100.0);

        SmartDashboard.putNumber("FL rpm for amp", shooterMotorFLEncoder.getVelocity());
        SmartDashboard.putNumber("FR rpm for amp", shooterMotorFREncoder.getVelocity());
       
    }

    public void runShooterHighSpeed() {

        shooterMotorFR.setVoltage(shooterVelVoltsFR);  // comment this out when running PID
        shooterMotorFL.setVoltage(shooterVelVoltsFL);  // comment this out when running PID

         // PID FL & FR Motor coefficients
         kP = 0.0;  //6e-5;   // REV suggested value. May need to change for our motors
         kI = 0;
         kD = 0; 
         kIz = 0; 
         kFF = 0.000015; // REV suggested value. May need to change for our motors
         kMaxOutputFL = 0.9; 
         kMinOutputFL= -0.9;
         kMaxOutputFR = 0.9; 
         kMinOutputFR= -0.9;
         maxRPM = 5676;  // from REV data sheet

        // set PID coefficients FL motor
        shooterPIDControllerFL.setP(kP);
        shooterPIDControllerFL.setP(kI);
        shooterPIDControllerFL.setP(kD);
        shooterPIDControllerFL.setIZone(kIz);
        shooterPIDControllerFL.setFF(kFF);
        shooterPIDControllerFL.setOutputRange(kMinOutputFL, kMaxOutputFL);

        // set PID coefficients FR motor
        shooterPIDControllerFR.setP(kP);
        shooterPIDControllerFR.setP(kI);
        shooterPIDControllerFR.setP(kD);
        shooterPIDControllerFR.setIZone(kIz);
        shooterPIDControllerFR.setFF(kFF);
        shooterPIDControllerFR.setOutputRange(kMinOutputFR, kMaxOutputFR);


        
        double flRPM =  -5320.0;  // TODO: get encoder values from smartdashboard
        double frRPM =  5205.0;  // TODO: get encoder values from smartdashboard

        //shooterPIDControllerFL.setReference(flRPM, CANSparkBase.ControlType.kVelocity);
        //shooterPIDControllerFR.setReference(frRPM, CANSparkBase.ControlType.kVelocity);


        SmartDashboard.putNumber("RPM FL Shooter", flRPM);
        SmartDashboard.putNumber("RPM FR Shooter", frRPM);
        SmartDashboard.putNumber("ProcessVariable Shooter FL", shooterMotorFLEncoder.getVelocity());
        SmartDashboard.putNumber("ProcessVariable Shooter FR", shooterMotorFREncoder.getVelocity());
       // Logger.registerCanSparkMax("Shooter Motor FL", () -> getVelocity());
       // Logger.registerCanSparkMax("Shooter Motor FR", shooterMotorFREncoder.getVelocity());
    }


    public void runShooterReverse() {
        double shooterVelVoltsReverseFR = Constants.FR_SHOOTER_MOTOR_REVERSE_PCT * 12.0;
        double shooterVelVoltsReverseFL = Constants.FL_SHOOTER_MOTOR_REVERSE_PCT * 12.0;
        shooterMotorFR.setVoltage(shooterVelVoltsReverseFR);
        shooterMotorFL.setVoltage(shooterVelVoltsReverseFL);
    }

    public void stopShooter() {
        shooterMotorFR.setVoltage(0.0);
        shooterMotorFL.setVoltage(0.0);
        
    }

    /* 
    * public void runLinearActuator() {
    *   linearActuatorMotor.setVoltage(linearActuatorVelVolts);
    * }
    */

    /* 
    * public void runLinearActuatorReverse() {
    *   linearActuatorMotor.setVoltage(linearActuatorVelVoltsReverse);
    * }
    */

   
    @Override
    public void periodic() {
        // TODO: Calculate average RPM and display on SmartDashboard.
        double flRPM = shooterMotorFLEncoder.getVelocity();
        double frRPM = shooterMotorFREncoder.getVelocity();
        shooterVelRPM = (flRPM + frRPM) / 2;
        // This method will be called once per scheduler run
    }

    public double getShooterVelRPM() {
        return shooterVelRPM;
    }
}
