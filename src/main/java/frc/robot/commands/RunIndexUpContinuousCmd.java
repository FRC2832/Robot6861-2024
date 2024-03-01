// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubSys;

public class RunIndexUpContinuousCmd extends Command {
    /** Creates a new RunIndexUp. */
    private final IndexerSubSys indexerSubSysObj;

    /**
     * Creates a new RunIndexUpCmd.
     * Makes indexerSubSysObj a requirement
     * 
     * @param indexerSubSysObj The IndexerSubsystem from the where it is being
     *                         called
     */

    public RunIndexUpContinuousCmd(IndexerSubSys indexerSubSysObj) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexerSubSysObj = indexerSubSysObj;
        addRequirements(indexerSubSysObj);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // No op
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexerSubSysObj.runIndexerUp();
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
