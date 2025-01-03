// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingSpeaker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterAnglerSubSys;

public class AngleShooterSafeZoneCmd extends Command {
  /** Creates a new AngleShooterAmpCmd. */
  private final ShooterAnglerSubSys shooterAnglerSubSysObj;

  private static final Timer TIMER = new Timer();
  private double timerMotor = 0.7;  //0.6 is about 41 deg. 0.375 for shooter rotates about 49 deg. works well for amp.  0.45 is 47deg.
  private double timerFinish = 7.0;

  public AngleShooterSafeZoneCmd(ShooterAnglerSubSys shooterAnglerSubSysObj) {
    this.shooterAnglerSubSysObj = shooterAnglerSubSysObj;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterAnglerSubSysObj);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TIMER.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (TIMER.get() < timerMotor) {
            shooterAnglerSubSysObj.runLinearActuatorReverse();
    } else {
        shooterAnglerSubSysObj.stopLinearActuator();
     }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //shooterAnglerSubSysObj.runLinearActuator();
    TIMER.stop();
    TIMER.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return TIMER.get() > timerFinish;
  }
}
