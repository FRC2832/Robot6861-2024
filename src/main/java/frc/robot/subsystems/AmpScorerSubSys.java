// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpScorerSubSys extends SubsystemBase {
  private final CANSparkMax ampMotor;
  private final RelativeEncoder ampEncoder;

  private double ampMotorVelPct;
  private double ampMotorReverseVelPct;
  private double ampMotorVolts;
  private double ampMotorReverseVolts;

  /** Creates a new AmpScorerSubSys. */
  public AmpScorerSubSys() {
    ampMotor = new CANSparkMax(Constants.AMP_MOTOR_CAN_ID, MotorType.kBrushless);
    ampMotor.setSmartCurrentLimit(Constants.AMP_MOTOR_SMART_CURRENT_LIMIT);
    ampMotor.setSecondaryCurrentLimit(Constants.AMP_MOTOR_SECONDARY_CURRENT_LIMIT);

    ampEncoder = ampMotor.getEncoder();

    ampMotorVelPct = Constants.AMP_UP_PCT / 100.0; // TODO: Possibly switch with AMP_DOWN_PCT
    ampMotorReverseVelPct = Constants.AMP_DOWN_PCT / 100.0;
    ampMotorVolts = ampMotorVelPct * 12.0;
    ampMotorReverseVolts = ampMotorReverseVelPct * 12.0;
  }

  public void runAmpMotor() {
    ampMotor.setVoltage(ampMotorVolts);
  }

  public void runAmpMotorReverse() {
    ampMotor.setVoltage(ampMotorReverseVolts);
  }

  public void stopAmpMotor() {
    ampMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Amp Encoder Position", ampEncoder.getPosition());
  }
}
