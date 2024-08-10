// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingSpeaker;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAnglerSubSys;

public class AngleShooterUpCmd extends Command {
    private final ShooterAnglerSubSys shooterAnglerSubSysObj;
    private static final Timer TIMER = new Timer();
    private final double timerLim;

    /** Creates a new AngleShooterUp. */
    public AngleShooterUpCmd(ShooterAnglerSubSys shooterAnglerSubSysObj) {
        this.shooterAnglerSubSysObj = shooterAnglerSubSysObj;
        timerLim = 3.0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterAnglerSubSysObj);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TIMER.restart();
        // No op
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //shooterAnglerSubSysObj.runLinearActuator();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //shooterAnglerSubSysObj.stopLinearActuator();
        TIMER.stop();
        TIMER.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return TIMER.get() > timerLim;
    }
}
