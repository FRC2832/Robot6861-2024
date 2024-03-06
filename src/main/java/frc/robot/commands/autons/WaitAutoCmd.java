// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitAutoCmd extends Command {
    private double waitTime;
    private static final Timer TIMER = new Timer();

    public WaitAutoCmd(double seconds) {
        waitTime = seconds;
    }

    @Override
    public void initialize() {
        TIMER.reset();
        TIMER.start();
    }

    @Override
    public void execute() {
        // just wait for the timer
    }

    @Override
    public void end(boolean interrupted) {
        TIMER.stop();
    }

    @Override
    public boolean isFinished() {
        return TIMER.get() >= waitTime;
    }
}
