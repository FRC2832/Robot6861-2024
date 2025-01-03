// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubSys;
import frc.robot.subsystems.ShooterAnglerSubSys;
import frc.robot.subsystems.ShooterSubSys;
import frc.robot.commands.ShootingSpeaker.AngleShooterBlackLineCmd;
import frc.robot.commands.ShootingSpeaker.AngleShooterUpCmd;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ShootBlackLineAutoCmd extends Command {
    private final ShooterSubSys shooterSubSysObj;
    private final ShooterAnglerSubSys shooterAnglerSubSysObj;
    private final IndexerSubSys indexerSubSysObj;
    private final double tgtShooterVelRPM;
    private static final double INDEXER_RUN_TIME = 1.0; // Length of time to run the indexer.
    private static final double PRIME_SHOOTER_TIME = 0.95;  //was 0.75
    private static final double ANGLER_SHOOTER_TIME = 0.35;
    private boolean isShooterPrimed; // When true, start the indexer.

    private static final Timer TIMER = new Timer();

    public ShootBlackLineAutoCmd(ShooterSubSys shooterSubSysObj, IndexerSubSys indexerSubSysObj, double tgtShooterVelRPM, ShooterAnglerSubSys shooterAnglerSubSysObj) {
        this.shooterSubSysObj = shooterSubSysObj;
        this.indexerSubSysObj = indexerSubSysObj;
        this.tgtShooterVelRPM = tgtShooterVelRPM;
        this.shooterAnglerSubSysObj = shooterAnglerSubSysObj;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubSysObj, indexerSubSysObj);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        TIMER.reset();
        TIMER.start();
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

        shooterSubSysObj.runShooterHighSpeed();
        /*if(TIMER.get() < ANGLER_SHOOTER_TIME && !isShooterPrimed) {
            shooterAnglerSubSysObj.runLinearActuatorReverse();
        } else {
            shooterAnglerSubSysObj.stopLinearActuator();
        }*/
        //CommandScheduler.getInstance().schedule(new AngleShooterBlackLineCmd(shooterAnglerSubSysObj));

        if (!isShooterPrimed && TIMER.get() >= PRIME_SHOOTER_TIME) {
            isShooterPrimed = true;
            TIMER.reset();
            TIMER.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubSysObj.stopShooter();
        indexerSubSysObj.stopIndexMotors();
        TIMER.stop();
        CommandScheduler.getInstance().schedule(new AngleShooterUpCmd(shooterAnglerSubSysObj));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return TIMER.get() >= INDEXER_RUN_TIME;
    }
}
