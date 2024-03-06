// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.livoniawarriors.swerve.SwerveDriveTrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveBackCmd extends Command {
    private final SwerveDriveTrain swerveObj;
    private final double tgtDistance;

    private double initDistance;
    private SwerveModulePosition[] curPos = new SwerveModulePosition[4];

    /** Creates a new RunIndexDown. */
    public DriveBackCmd(SwerveDriveTrain swerve, double distance) {
        this.swerveObj = swerve;
        this.tgtDistance = distance;
        this.initDistance = swerveObj.getSwervePositions()[0].distanceMeters;
        addRequirements(swerve);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        curPos = swerveObj.getSwervePositions();
        initDistance = curPos[0].distanceMeters;
        // Add distance to finalPos
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Might need to change value of yspeed (second parameter) to negative
        swerveObj.swerveDrive(0, -0.3, 0); // Drive backwards at 30% speed
        curPos = swerveObj.getSwervePositions();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        /*
         * Stops swerve drive motors when command ends
         * public void stopWheels() {
         * swerveDrive(0, 0, 0);
         * }
         * From SwerveDriveTrain.java in src/main/java/org/livoniawarriors/swerve
         */
        swerveObj.stopWheels();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(curPos[0].distanceMeters - initDistance) >= Math.abs(tgtDistance);
    }
}
