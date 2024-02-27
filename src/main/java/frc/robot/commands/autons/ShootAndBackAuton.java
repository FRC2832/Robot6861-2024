// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.livoniawarriors.swerve.SwerveDriveTrain;
import frc.robot.subsystems.ShooterSubSys;
import frc.robot.subsystems.PracticeSwerveHw;
import frc.robot.commands.PrimeShooterCmd;
import frc.robot.commands.DriveBackCmd;


public class ShootAndBackAuton extends SequentialCommandGroup {
    public ShootAndBackAuton(SwerveDriveTrain swerveObj, ShooterSubSys shooterObj) {
        int distance = 2;
        addCommands(
            new PrimeShooterCmd(shooterObj),
            new DriveBackCmd(swerveObj, distance)
        );
    }
}
