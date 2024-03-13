// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubSys;

public class RunIndexDownCmd extends Command {
    private final IndexerSubSys indexerSubSysObj;

    /** Creates a new RunIndexDown. */
    public RunIndexDownCmd(IndexerSubSys indexerSubSys) {
        this.indexerSubSysObj = indexerSubSys;
        addRequirements(indexerSubSys);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // No op
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexerSubSysObj.runIndexerDown();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexerSubSysObj.stopIndexMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
