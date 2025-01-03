// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootingSpeaker;

import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterAnglerSubSys;

public class AngleShooterDownCmd extends Command {
    private final ShooterAnglerSubSys shooterAnglerSubSysObj;
    //private static final Timer TIMER = new Timer();
    //private double timerMotor = 0.6;   //shooter about 42 deg.
    //private double timerFinish = 7.0;

    /** Creates a new AngleShooterDown. */
    public AngleShooterDownCmd(ShooterAnglerSubSys shooterAnglerSubSysObj) {
        this.shooterAnglerSubSysObj = shooterAnglerSubSysObj;
        addRequirements(shooterAnglerSubSysObj);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //TIMER.restart();
    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("running shooter angle");
        shooterAnglerSubSysObj.angleShooterDown();
        /*if (TIMER.get() < timerMotor) {
            shooterAnglerSubSysObj.runLinearActuatorReverse(); //angle shooter down to about 42 deg.
        } else {
            shooterAnglerSubSysObj.stopLinearActuator();
        }*/
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterAnglerSubSysObj.angleShooterUp();
        //shooterAnglerSubSysObj.stopAngledShooterMotor();   


        
       // shooterAnglerSubSysObj.runLinearActuator();  // angle shooter back to 60 deg.
        //TIMER.stop();
        //TIMER.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
        //return TIMER.get() > timerFinish;
    }
}
