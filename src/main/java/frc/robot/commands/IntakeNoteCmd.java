// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.livoniawarriors.REVColorSensor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubSys;

public class IntakeNoteCmd extends Command {
    private boolean isThresholdCrossed;
    private final IntakeSubSys intakeSubSysObj;
    private final REVColorSensor colorSensorObj;
    private static final Timer TIMER = new Timer();
    private static final double MAX_RUN_TIME = 3.5; // seconds
    private static final double PROX_THRESHOLD = 0.5;

    /**
     * Creates a new IntakeNote.
     * Makes intakeSubSysObj a requirement
     * 
     * @param intakeSubSysObj The IntakeSubsystem from the where it is being called
     */
    public IntakeNoteCmd(IntakeSubSys intakeSubSys, REVColorSensor colorSensorObj) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intakeSubSysObj = intakeSubSys;
        this.colorSensorObj = colorSensorObj;
        addRequirements(intakeSubSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TIMER.reset();
        TIMER.start();
        isThresholdCrossed = false;
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
        TIMER.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Color color = colorSensorObj.getColor();
        double prox = colorSensorObj.getProximity();
        if (!isThresholdCrossed && prox >= PROX_THRESHOLD) {
            isThresholdCrossed = true;
            return false;
        }
        if (isThresholdCrossed) {
            return prox < PROX_THRESHOLD || TIMER.get() >= MAX_RUN_TIME;
        }
        return TIMER.get() >= MAX_RUN_TIME;
        // return prox >= PROX_THRESHOLD || TIMER.get() >= MAX_RUN_TIME;
        // return color == Color.kOrange || color == Color.kOrangeRed || TIMER.get() >=
        // MAX_RUN_TIME;
        // return TIMER.get() >= MAX_RUN_TIME;
        // REV color sensor
    }
}
