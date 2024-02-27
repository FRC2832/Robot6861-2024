// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons.blue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.livoniawarriors.swerve.SwerveDriveTrain;

public class ShootAndBackAuton extends SequentialCommandGroup {
    public ShootAndBackAuton(SwerveDriveTrain swerveObj, ShooterSubSys shooterObj) {
        int distance = 2;
        addCommands(
            new PrimeShooterCmd(shooterObj),
            new DriveBackCmd(drivetrainObj, distance)
        );
    }
}
