// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSubSys extends SubsystemBase {
    /** Creates a new Intake. */
    // if Operator right trigger held, run intake motors CAN Id 1 & 2

    // Neo 550 motors
    // Intake in is positive volts
    // eject is negative volts
    // once note is fully in indexer, run intake backwards

    private final CANSparkMax intakeMotor;
    //private SparkPIDController intakePidController;
    //private RelativeEncoder intakeEncoder;
    private double intakeVelVolts;
    private double outtakeVelVolts;
    private double intakeVelPct;
    private double outtakeVelPct;
    //private DigitalInput intakeSensor;

    //private double kP;
    //private double kI;
    //private double kD;
   // private double kIz;
   // private double kFF;
   // private double kMaxOutput;
   // private double kMinOutput;
   // private double maxRPM;

    public IntakeSubSys() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        intakeMotor.setSmartCurrentLimit(Constants.INTAKE_MOTOR_SMART_CURRENT_LIMIT);
        intakeMotor.setSecondaryCurrentLimit(Constants.INTAKE_MOTOR_SECONDARY_CURRENT_LIMIT);
        
        //from https://www.revrobotics.com/development-spark-max-users-manual/#section-3-3-2-1
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);  //to help reduce CANbus high utilization
        intakeMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);  // TODO: might be able to go higher than 100....


        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeVelPct = Constants.INTAKE_MOTOR_PCT / 100.0;
        outtakeVelPct = Constants.OUTTAKE_MOTOR_PCT / 100.0;
        intakeVelVolts = intakeVelPct * 12.0;
        outtakeVelVolts = outtakeVelPct * 12.0;
    
        //intakeSensor = new DigitalInput(Constants.INTAKE_SENSOR_DIO_PORT);

        //intakeEncoder = intakeMotor.getEncoder();
        //intakePidController = intakeMotor.getPIDController();
    }

    // Runs the IntakeMotors in a positive direction(Inwards)
    public void runIntake() {
       
        intakeMotor.setVoltage(intakeVelVolts);
        SmartDashboard.putNumber("Intake Volts", intakeVelVolts);

        /* 
        kP = 0.0; // Suggested Value: 0
        kI = 0.0;
        kD = 0.0;
        kIz = 0.0;
        kFF = 0.0; // Suggested Value: 0.000015
        kMaxOutput = 0.95;
        kMinOutput = -0.95;
        maxRPM = 5700.0; // Suggested Value: 5700

        intakePidController.setP(kP);
        intakePidController.setI(kI);
        intakePidController.setD(kD);
        intakePidController.setIZone(kIz);
        intakePidController.setFF(kFF);
        intakePidController.setOutputRange(kMinOutput, kMaxOutput);
        
        */
        // double setPoint = m_stick.getY()*maxRPM;
        //intakePidController.setReference(maxRPM, CANSparkMax.ControlType.kVelocity);

        // SmartDashboard.putNumber("SetPoint", setPoint);
        //SmartDashboard.putNumber("ProcessVariable", intakeEncoder.getVelocity());
    }

    // Runs the IntakeMotors in a negative direction(Outwards)
    public void runOuttake() {
        intakeMotor.setVoltage(outtakeVelVolts);
    }

    public void stopIntakeMotors() {
        intakeMotor.setVoltage(0.0);
    }

    /*public boolean getIntakeSensor() {
        return !intakeSensor.get();
    }*/

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.putNumber("Intake Motor Encoder", intakeEncoder.getVelocity());
        //SmartDashboard.putBoolean("Intake Sensor", !intakeSensor.get());
    }
}
