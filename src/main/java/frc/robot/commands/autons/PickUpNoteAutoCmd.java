// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import org.livoniawarriors.REVColorSensor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubSys;
import frc.robot.subsystems.IntakeSubSys;

public class PickUpNoteAutoCmd extends Command {
    private boolean isThresholdCrossed;
    private final IntakeSubSys intakeSubSysObj;
    private final IndexerSubSys indexerSubSysObj;
    private final REVColorSensor colorSensorObj;
    private static final Timer TIMER = new Timer();
    private static final Timer BUFFER_TIMER = new Timer();
    private static final double AUTON_MAX_RUN_TIME = 6.0;
    private static final double BUFFER_TIME = 0.5;
    private static final double PROX_THRESHOLD = 0.5;

    public PickUpNoteAutoCmd(IntakeSubSys intakeSubSysObj, IndexerSubSys indexerSubSysObj, REVColorSensor colorSensorObj) {
        this.intakeSubSysObj = intakeSubSysObj;
        this.indexerSubSysObj = indexerSubSysObj;
        this.colorSensorObj = colorSensorObj;
        addRequirements(intakeSubSysObj, indexerSubSysObj);
    }

    @Override
    public void initialize() {
        TIMER.reset();
        TIMER.start();
        BUFFER_TIMER.reset();
        isThresholdCrossed = false;
    }

    @Override
    public void execute() {
        intakeSubSysObj.runIntake();
        indexerSubSysObj.runIndexerUp();
        if (isThresholdCrossed) {
            BUFFER_TIMER.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubSysObj.stopIntakeMotors();
        indexerSubSysObj.stopIndexMotors();
        TIMER.stop();
        BUFFER_TIMER.stop();
    }

    @Override
    public boolean isFinished() {
        double prox = colorSensorObj.getProximity();
        if (!isThresholdCrossed && prox >= PROX_THRESHOLD) {
            // top of note passes sensor
            isThresholdCrossed = true;
            return false;
        }
        if (isThresholdCrossed) {
            return (prox < PROX_THRESHOLD && BUFFER_TIMER.get() >= BUFFER_TIME) || TIMER.get() >= AUTON_MAX_RUN_TIME;
        }
        return TIMER.get() >= AUTON_MAX_RUN_TIME;
    }
}
