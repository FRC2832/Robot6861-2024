// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubSys;

public class RunIndexUpCmd extends Command {
    /** Creates a new RunIndexUp. */
    private final IndexerSubSys indexerSubSysObj;
    private static final Timer TIMER = new Timer();
    private static final double MAX_RUN_TIME = 2.5; // TODO: confirm this time

    /**
     * Creates a new RunIndexUpCmd.
     * Makes indexerSubSysObj a requirement
     * 
     * @param indexerSubSysObj The IndexerSubsystem from the where it is being
     *                         called
     */

    public RunIndexUpCmd(IndexerSubSys indexerSubSysObj) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexerSubSysObj = indexerSubSysObj;
        addRequirements(indexerSubSysObj);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TIMER.reset();
        TIMER.start();
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
        return TIMER.get() >= MAX_RUN_TIME;
    }
}
