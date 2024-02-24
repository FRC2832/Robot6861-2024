// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.livoniawarriors.REVColorSensor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubSys;

public class RunIndexUpCmd extends Command {
    /** Creates a new RunIndexUp. */
    private boolean isThresholdCrossed;
    private final IndexerSubSys indexerSubSysObj;
    private final REVColorSensor colorSensorObj;
    private static final Timer BUFFER_TIMER = new Timer();
    private static final Timer TIMER = new Timer();
    private static final double MAX_RUN_TIME = 3.5; // TODO: confirm this time
    private static final double BUFFER_TIME = 1.0;
    private static final double PROX_THRESHOLD = 0.5;

    /**
     * Creates a new RunIndexUpCmd.
     * Makes indexerSubSysObj a requirement
     * 
     * @param indexerSubSysObj The IndexerSubsystem from the where it is being
     *                         called
     */

    public RunIndexUpCmd(IndexerSubSys indexerSubSysObj, REVColorSensor colorSensorObj) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.indexerSubSysObj = indexerSubSysObj;
        this.colorSensorObj = colorSensorObj;
        addRequirements(indexerSubSysObj);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TIMER.reset();
        TIMER.start();
        BUFFER_TIMER.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        indexerSubSysObj.runIndexerUp();
        if (isThresholdCrossed) {
            BUFFER_TIMER.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        indexerSubSysObj.stopIndexMotors();
        TIMER.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Color color = colorSensorObj.getColor();
        // return color == Color.kOrange || color == Color.kOrangeRed || TIMER.get() >=
        // MAX_RUN_TIME;
        double prox = colorSensorObj.getProximity();
        if (!isThresholdCrossed && prox >= PROX_THRESHOLD) {
            isThresholdCrossed = true;
            return false;
        }
        if (isThresholdCrossed) {
            return (prox < PROX_THRESHOLD && BUFFER_TIMER.get() >= BUFFER_TIME) || TIMER.get() >= MAX_RUN_TIME;
        }
        return false;
        // double prox = colorSensorObj.getProximity();
        // return prox >= PROX_THRESHOLD || TIMER.get() >= MAX_RUN_TIME;

        // return colorSensorObj.getProximity() >= 0.5;
        // return TIMER.get() >= MAX_RUN_TIME;

    }
}
