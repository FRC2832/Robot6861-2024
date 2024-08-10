// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubSys extends SubsystemBase {
    /** Creates a new Indexer. */

    // Run Indexer sequential to Intake trigger - Operator right trigger

    // Neo 550 motors
    // Test for which direction positive/negative input causes?
    // Intake in is positive volts
    // eject is negative volts
    // once note is fully in indexer, stop intake motor unitl driver presses driver
    // right trigger for shooting

    private final CANSparkMax indexMotor;
    private double upIndexVelVolts;
    private double downIndexVelVolts;
    private double upIndexVelPct;
    private double downIndexVelPct;

    public IndexerSubSys() {
        indexMotor = new CANSparkMax(Constants.INDEX_MOTOR_CAN_ID, MotorType.kBrushless);
        indexMotor.setSmartCurrentLimit(Constants.INDEX_MOTOR_SMART_CURRENT_LIMIT);
        indexMotor.setSecondaryCurrentLimit(Constants.INDEX_MOTOR_SECONDARY_CURRENT_LIMIT);
        
        //from https://www.revrobotics.com/development-spark-max-users-manual/#section-3-3-2-1
        indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);  //to help reduce CANbus high utilization
        indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);  // TODO: might be able to go higher than 100....
        indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
        indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
        indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 100);
        indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 300);
        indexMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 300);

        indexMotor.setIdleMode(IdleMode.kBrake);

        upIndexVelPct = Constants.UPINDEX_MOTOR_PCT / 100.0;
        downIndexVelPct = Constants.DOWNINDEX_MOTOR_PCT / 100.0;
        upIndexVelVolts = upIndexVelPct * 12.0;
        downIndexVelVolts = downIndexVelPct * 12.0;
    }

    // Runs the IntakeMotors in a positive direction(Inwards)
    public void runIndexerUp() {
        indexMotor.setVoltage(upIndexVelVolts);
    }

    // Runs the IntakeMotors in a negative direction(Outwards)
    public void runIndexerDown() {
        indexMotor.setVoltage(downIndexVelVolts);
    }

    public void stopIndexMotors() {
        indexMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
