// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    private double lowKpFl;    
    private double lowKiFl;
    private double lowKdFl;
    private double lowKizFl; 
    private double lowKffFl;
    private double lowKpFr;  
    private double lowKiFr;
    private double lowKdFr; 
    private double lowKizFr; 
    private double lowKffFr;


    private double kPFl;
    private double kIFl;
    private double kDFl;
    private double kIzFl;
    private double kFFFl;
    private double kMaxOutputFL;
    private double kMinOutputFL;
    private double kPFr;
    private double kIFr;
    private double kDFr;
    private double kIzFr;
    private double kFFFr;
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
        //shooterMotorFL.set(Constants.FL_SHOOTER_MOTOR_PCT_AMP / 100.0);
        //shooterMotorFR.set(Constants.FR_SHOOTER_MOTOR_PCT_AMP / 100.0);


         // PID FL  Motor coefficients
         lowKpFl = 0.0256;    
         lowKiFl = 0.0;
         lowKdFl = 0.0; 
         lowKizFl = 0.0; 
         lowKffFl = 0.00016; 
         kMaxOutputFL = 0.98; 
         kMinOutputFL = -0.98;
         maxRPM = 5676.0;  // from REV data sheet

        // set PID coefficients FL motor
        shooterPIDControllerFL.setP(lowKpFl);
        shooterPIDControllerFL.setP(lowKiFl);
        shooterPIDControllerFL.setP(lowKdFl);
        shooterPIDControllerFL.setIZone(lowKizFl);
        shooterPIDControllerFL.setFF(lowKffFl);
        shooterPIDControllerFL.setOutputRange(kMinOutputFL, kMaxOutputFL);

        // PID FR  Motor coefficients
         lowKpFr = 0.0256;  
         lowKiFr = 0.0;
         lowKdFr = 0.0; 
         lowKizFr = 0.0; 
         lowKffFr = 0.00016; 
         kMaxOutputFR = 0.98; 
         kMinOutputFR = -0.98;
         maxRPM = 5676.0;  // from REV data sheet


        // set PID coefficients FR motor
        shooterPIDControllerFR.setP(lowKpFr);
        shooterPIDControllerFR.setP(lowKiFr);
        shooterPIDControllerFR.setP(lowKdFr);
        shooterPIDControllerFR.setIZone(lowKizFr);
        shooterPIDControllerFR.setFF(lowKffFr);
        shooterPIDControllerFR.setOutputRange(kMinOutputFR, kMaxOutputFR);


        
        double lowFlRPM =  -3000.0;  
        double lowFrRPM =  3000.0;  

        shooterPIDControllerFL.setReference(lowFlRPM, CANSparkMax.ControlType.kVelocity);
        shooterPIDControllerFR.setReference(lowFrRPM, CANSparkMax.ControlType.kVelocity);


        SmartDashboard.putNumber("RPM FL Shooter", lowFlRPM);
        SmartDashboard.putNumber("RPM FR Shooter", lowFrRPM);
        SmartDashboard.putNumber("ProcessVariable Shooter FL", shooterMotorFLEncoder.getVelocity());
        SmartDashboard.putNumber("ProcessVariable Shooter FR", shooterMotorFREncoder.getVelocity());

        SmartDashboard.putNumber("FL rpm for amp", shooterMotorFLEncoder.getVelocity());
        SmartDashboard.putNumber("FR rpm for amp", shooterMotorFREncoder.getVelocity());
       
    }

    public void runShooterHighSpeed() {

        // shooterMotorFR.setVoltage(shooterVelVoltsFR);  // comment this out when running PID
        // shooterMotorFL.setVoltage(shooterVelVoltsFL);  // comment this out when running PID

         // PID FL  Motor coefficients
         kPFl = 0.0016;    
         kIFl = 0.0;
         kDFl = 0.0; 
         kIzFl = 0.0; 
         kFFFl = 0.00040; 
         kMaxOutputFL = 0.98; 
         kMinOutputFL = -0.98;
         maxRPM = 5676.0;  // from REV data sheet

        // set PID coefficients FL motor
        shooterPIDControllerFL.setP(kPFl);
        shooterPIDControllerFL.setP(kIFl);
        shooterPIDControllerFL.setP(kDFl);
        shooterPIDControllerFL.setIZone(kIzFl);
        shooterPIDControllerFL.setFF(kFFFl);
        shooterPIDControllerFL.setOutputRange(kMinOutputFL, kMaxOutputFL);

        // PID FR  Motor coefficients
         kPFr = 0.0;  //6e-5;   // REV suggested value. May need to change for our motors
         kIFr = 0.0;
         kDFr = 0.0; 
         kIzFr = 0.0; 
         kFFFr = 0.000182; // REV suggested value. May need to change for our motors
         kMaxOutputFR = 0.98; 
         kMinOutputFR = -0.98;
         maxRPM = 5676.0;  // from REV data sheet


        // set PID coefficients FR motor
        shooterPIDControllerFR.setP(kPFr);
        shooterPIDControllerFR.setP(kIFr);
        shooterPIDControllerFR.setP(kDFr);
        shooterPIDControllerFR.setIZone(kIzFr);
        shooterPIDControllerFR.setFF(kFFFr);
        shooterPIDControllerFR.setOutputRange(kMinOutputFR, kMaxOutputFR);


        
        double flRPM =  -5350.0;  // TODO: was 5320.  Might need 2 speeds for far and away. get encoder values from smartdashboard
        double frRPM =  5300.0;  // TODO: was 5205.  get encoder values from smartdashboard

        shooterPIDControllerFL.setReference(flRPM, CANSparkMax.ControlType.kVelocity);
        shooterPIDControllerFR.setReference(frRPM, CANSparkMax.ControlType.kVelocity);


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
        
        shooterMotorFR.setIdleMode(IdleMode.kBrake); 
        shooterMotorFL.setIdleMode(IdleMode.kBrake);
        
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
