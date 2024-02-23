// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubSys;

public class IntakeNoteCmd extends Command {
    private final IntakeSubSys intakeSubSysObj;
    private static final Timer TIMER = new Timer();
    private static final double MAX_RUN_TIME = 2.0; //seconds

    /**
     * Creates a new IntakeNote.
     * Makes intakeSubSysObj a requirement
     * 
     * @param intakeSubSysObj The IntakeSubsystem from the where it is being called
     */
    public IntakeNoteCmd(IntakeSubSys intakeSubSys) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intakeSubSysObj = intakeSubSys;
        addRequirements(intakeSubSys);
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
        intakeSubSysObj.runIntake();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubSysObj.stopIntakeMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // REV color sensor
        return TIMER.get() >= MAX_RUN_TIME;
    }
}
