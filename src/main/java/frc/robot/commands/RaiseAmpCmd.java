// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpScorerSubSys;

public class RaiseAmpCmd extends Command {
  private final AmpScorerSubSys ampScorerSubSysObj;

  /** Creates a new RaiseAmpCmd. */
  public RaiseAmpCmd(AmpScorerSubSys ampScorerSubSysObj) {
    this.ampScorerSubSysObj = ampScorerSubSysObj;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampScorerSubSysObj);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampScorerSubSysObj.runAmpMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampScorerSubSysObj.stopAmpMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
