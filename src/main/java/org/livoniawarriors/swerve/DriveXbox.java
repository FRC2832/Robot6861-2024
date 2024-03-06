package org.livoniawarriors.swerve;

import org.livoniawarriors.UtilFunctions;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/**
 * Drive the robot with xbox controller
 */
public class DriveXbox extends Command {
    private SwerveDriveTrain drive;
    private XboxController cont;
    private DoubleSubscriber deadband;

    /**
     * Inject the drivetrain and controller to use
     * 
     * @param drive Drivetrain to command
     * @param cont  Controller to read from
     */
    public DriveXbox(SwerveDriveTrain drive, CommandXboxController cont) {
        this.drive = drive;
        this.cont = cont.getHID();
        deadband = UtilFunctions.getSettingSub("DriveXbox/Deadband", 0.13);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.swerveDrive(0, 0, 0, false);
    }

    @Override
    public void execute() {
        // driver clicked field reset stick
        if (cont.getStartButtonPressed()) { //getLeftStickButtonPressed()
            drive.resetFieldOriented();
        }

        double dead = deadband.get();
        double xSpeed = UtilFunctions.deadband(-cont.getLeftY(), dead);
        double ySpeed = UtilFunctions.deadband(-cont.getLeftX(), dead);
        double turn = UtilFunctions.deadband(-cont.getRightX(), dead);

        double newXSpeed;
        double newYSpeed;
        if (cont.getLeftBumper()) { // snail mode
            SmartDashboard.putString("Drive mode:", "Snail");
            newXSpeed = Constants.SNAIL_MODE * xSpeed * drive.getMaxDriverSpeed();
            newYSpeed = Constants.SNAIL_MODE * ySpeed * drive.getMaxDriverSpeed();
            SmartDashboard.putNumber("xSpeed:", newXSpeed);
            SmartDashboard.putNumber("ySpeed:", newYSpeed);
            drive.swerveDrive(
                    newXSpeed,
                    newYSpeed,
                    turn * drive.getMaxDriverOmega());
        } else if (cont.getLeftTriggerAxis() >= 0.5) { // turtle mode
            SmartDashboard.putString("Drive mode:", "Turtle");
            newXSpeed = Constants.TURTLE_MODE * xSpeed * drive.getMaxDriverSpeed();
            newYSpeed = Constants.TURTLE_MODE * ySpeed * drive.getMaxDriverSpeed();
            drive.swerveDrive(
                    newXSpeed,
                    newYSpeed,
                    turn * drive.getMaxDriverOmega());
        } else {
            SmartDashboard.putString("Drive mode:", "Normal");
            newXSpeed = xSpeed * drive.getMaxDriverSpeed();
            newYSpeed = ySpeed * drive.getMaxDriverSpeed();
            drive.swerveDrive(
                    newXSpeed,
                    newYSpeed,
                    turn * drive.getMaxDriverOmega());
        }
        SmartDashboard.putNumber("xSpeed:", newXSpeed);
        SmartDashboard.putNumber("ySpeed:", newYSpeed);
    }

    @Override
    public boolean isFinished() {
        // never end
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // No op
    }
}
