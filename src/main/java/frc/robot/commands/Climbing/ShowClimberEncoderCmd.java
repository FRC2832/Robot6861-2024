// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubSys;

public class ShowClimberEncoderCmd extends Command {
    /** Creates a new ShowClimberEncoder. */
    private ClimberSubSys climberSubSysObj;

    public ShowClimberEncoderCmd(ClimberSubSys climberSubSysObj) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.climberSubSysObj = climberSubSysObj;
        addRequirements(climberSubSysObj);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        climberSubSysObj.resetEncoders();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climberSubSysObj.showEncoders();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // No op
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
