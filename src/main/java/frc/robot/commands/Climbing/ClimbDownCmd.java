// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubSys;

public class ClimbDownCmd extends Command {
    /** Creates a new ClimbDownCmd. */
    private final ClimberSubSys climberSubSysObj;

    /**
     * Creates a new Climb Up Command.
     * Makes ClimberSubSysObj a requirement
     * 
     * @param ClimberSubSysObj The ClimberSubsystem from the where it is being
     *                         called
     */

    public ClimbDownCmd(ClimberSubSys climberSubSys) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.climberSubSysObj = climberSubSys;
        addRequirements(climberSubSysObj);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // No op
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climberSubSysObj.runClimberDown();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climberSubSysObj.stopClimberMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
