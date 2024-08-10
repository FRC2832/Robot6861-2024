// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants;

public class ShooterAnglerSubSys extends SubsystemBase {
    private final CANSparkMax angledShooterMotor;
    private final RelativeEncoder angledShooterEncoder;
    private SparkPIDController angledShooterPIDController;
    private double kP;
    private double kI;
    private double kD;
    private double kIz;
    private double kFF;
    private double kMaxOutput;
    private double kMinOutput;
    //private static final Timer TIMER = new Timer();
    //private final double timerLim;
    //private final double timerLimAuton;
    //private double linearActuatorVelVolts;
    //private double linearActuatorVelPct;
    //private double linearActuatorVelReversePct;
    //private double linearActuatorVelVoltsReverse;

    /** Creates a new ShooterAnglerSubsys. */
    public ShooterAnglerSubSys() {
        // Rev NEo 550 
        angledShooterMotor = new CANSparkMax(Constants.ANGLED_SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
        angledShooterMotor.setSmartCurrentLimit(Constants.ANGLED_SHOOTER_MOTOR_SMART_CURRENT_LIMIT);
        angledShooterMotor.setSecondaryCurrentLimit(Constants.ANGLED_SHOOTER_MOTOR_SECONDARY_CURRENT_LIMIT);


        
        //from https://www.revrobotics.com/development-spark-max-users-manual/#section-3-3-2-1
        angledShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);  //to help reduce CANbus high utilization
        angledShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  // TODO: might be able to go higher than 100....
        angledShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20); 
        angledShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50); 
        angledShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);
        angledShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
        angledShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);

        angledShooterEncoder = angledShooterMotor.getEncoder(); 
        angledShooterEncoder.setPosition(0.0);

        angledShooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);


        //linearActuatorVelPct = Constants.LINEAR_ACTUATOR_MOTOR_PCT / 100.0;
        //linearActuatorVelReversePct = Constants.LINEAR_ACTUATOR_MOTOR_REVERSE_PCT / 100.0;
        //linearActuatorVelVolts = linearActuatorVelPct * 12.0;
        //linearActuatorVelVoltsReverse = linearActuatorVelReversePct * 12.0;


        //timerLim = 1.75;
        //timerLimAuton = 0.4;
    }


    public void resetEncoders() {
        angledShooterEncoder.setPosition(0.0);
    }

    public double showEncoders() {
        return angledShooterEncoder.getPosition();
    }

     public void angleShooterDown() {
        // Uncomment this for development, testing or debugging work:
        SmartDashboard.putNumber("Angled Shooter encoder - down", angledShooterEncoder.getPosition());

        // PID coefficients
        kP = 0.001;  //TODO: was 0.5 for climber
          //kI = 0.0;
          //kD = 0.0;
          //kIz = 0.0;
          //kFF = 0.0;
        kMaxOutput = 0.20;  //was .98 for climber
        kMinOutput = -0.10; //was -.98 for climber

        // set PID coefficients
        angledShooterPIDController.setP(kP);
           //climberPIDController.setI(kI);
           //climberPIDController.setD(kD);
           //climberPIDController.setIZone(kIz);
        //climberPIDController.setFF(kFF);
        angledShooterPIDController.setOutputRange(kMinOutput, kMaxOutput);

        double rotations = 6.0;  //TODO: determine this value.  50 or -50?

            // climberMotor.setVoltage(downClimbVelVolts);

        angledShooterPIDController.setReference(rotations, CANSparkBase.ControlType.kPosition);

        // Uncomment these for development, testing or debugging work:
        SmartDashboard.putNumber("SetPoint", rotations);
        SmartDashboard.putNumber("ProcessVariable", angledShooterEncoder.getPosition());

    }

    public void stopAngledShooterMotor() {
        angledShooterMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Uncomment this for development, testing or debugging work:
        SmartDashboard.putNumber("Angled Shooter encoder", angledShooterEncoder.getPosition());
    }

    /* public void runShooterDown() {
        TIMER.restart();
        if (TIMER.get() < timerLim) {
            linearActuatorMotor.setVoltage(linearActuatorVelVolts);
        } else {
            linearActuatorMotor.setVoltage(0.0);
        }
    
        TIMER.stop();
        TIMER.reset();
    }*/


    /*public void runLinearActuatorAuton() {
        
        linearActuatorMotor.setVoltage(linearActuatorVelVolts); 
    
    }


    public void runLinearActuatorReverse() {
        linearActuatorMotor.setVoltage(linearActuatorVelVoltsReverse);
    }

    public void stopLinearActuator() {
        linearActuatorMotor.setVoltage(0.0);
    }*/

}
