// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.awt.Color;

import org.livoniawarriors.ColorHSV;
import org.livoniawarriors.REVColorSensor;
import org.livoniawarriors.leds.ILedSubsystem;
import org.livoniawarriors.leds.LightningFlash;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootingSpeaker.AngleShooterUpCmd;
import frc.robot.subsystems.IntakeSubSys;

public class IntakeNoteCmd extends Command {   
    private boolean isThresholdCrossed;
    private final IntakeSubSys intakeSubSysObj;
    private final REVColorSensor colorSensorObj;
    //private final ILedSubsystem leds;
    private final XboxController driverController;
    private final XboxController operatorController;
    private static final Timer TIMER = new Timer();
    private static final double MAX_RUN_TIME = 7.0; // was 5.0 seconds
    private static final double PROX_THRESHOLD = 0.7;

    /**
     * Creates a new IntakeNote.
     * Makes intakeSubSysObj a requirement
     * 
     * @param intakeSubSysObj The IntakeSubsystem from the where it is being called
     */
    public IntakeNoteCmd(IntakeSubSys intakeSubSys, REVColorSensor colorSensorObj,
    CommandXboxController driverController, CommandXboxController operatorController) {  //ILedSubsystem leds,
        // Use addRequirements() here to declare subsystem dependencies.
        this.intakeSubSysObj = intakeSubSys;
        this.colorSensorObj = colorSensorObj;
        //this.leds = leds;
        this.driverController = driverController.getHID();
        this.operatorController = operatorController.getHID();
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
        if (isThresholdCrossed) {
            driverController.setRumble(RumbleType.kBothRumble, 0.8);
            operatorController.setRumble(RumbleType.kBothRumble, 0.8);

           //CommandScheduler.getInstance().schedule(new LightningFlash(leds, Color.kDarkSalmon);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSubSysObj.stopIntakeMotors();
        driverController.setRumble(RumbleType.kBothRumble, 0.0);
        operatorController.setRumble(RumbleType.kBothRumble, 0.0);
        TIMER.stop();
        //driverController.setRumble(RumbleType.kBothRumble, 0);
        //operatorController.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Color color = colorSensorObj.getColor();
        double prox = colorSensorObj.getProximity();
        if (!isThresholdCrossed && prox >= PROX_THRESHOLD) {
            isThresholdCrossed = true;
            //color == Color.kOrange;
            return false;
        }
        if (isThresholdCrossed) {
            return prox < PROX_THRESHOLD || TIMER.get() >= MAX_RUN_TIME;
        }
        //return TIMER.get() >= MAX_RUN_TIME;
        return prox >= PROX_THRESHOLD || TIMER.get() >= MAX_RUN_TIME;
        // return color == Color.kOrange || color == Color.kOrangeRed || TIMER.get() >=
        // MAX_RUN_TIME;
        // return TIMER.get() >= MAX_RUN_TIME;
        // REV color sensor
    }
}
