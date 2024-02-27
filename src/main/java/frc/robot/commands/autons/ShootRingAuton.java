// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubSys;
import frc.robot.subsystems.ShooterSubSys;

public class ShootRingAuton extends Command {
    private final ShooterSubSys shooterSubSysObj;
    private final IndexerSubSys indexerSubSysObj;
    private final double tgtShooterVelRPM;
    private static final double INDEXER_RUN_TIME = 2.5; // Length of time to run the indexer.
    private boolean isShooterPrimed; // When true, start the indexer.

    private static final Timer TIMER = new Timer();

    /** Creates a new ShootRingAuton. */
    public ShootRingAuton(ShooterSubSys shooterSubSysObj, IndexerSubSys indexerSubSysObj, double tgtShooterVelRPM) {
        this.shooterSubSysObj = shooterSubSysObj;
        this.indexerSubSysObj = indexerSubSysObj;
        this.tgtShooterVelRPM = tgtShooterVelRPM;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubSysObj, indexerSubSysObj);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TIMER.reset();
        isShooterPrimed = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isShooterPrimed) {
            indexerSubSysObj.runIndexerUp();
        } else {
            indexerSubSysObj.stopIndexMotors();
        }
        shooterSubSysObj.runShooter();
        if (shooterSubSysObj.getShooterVelRPM() - tgtShooterVelRPM <= 0.1) {
            isShooterPrimed = true;
            TIMER.start();
        } else {
            isShooterPrimed = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubSysObj.stopShooter();
        indexerSubSysObj.stopIndexMotors();
        TIMER.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return TIMER.get() >= INDEXER_RUN_TIME;
    }
}
