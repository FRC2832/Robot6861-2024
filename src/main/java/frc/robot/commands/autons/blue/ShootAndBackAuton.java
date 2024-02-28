// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons.blue;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveBackCmd;
import frc.robot.commands.autons.ShootRingAuton;
import frc.robot.subsystems.ShooterSubSys;
import frc.robot.subsystems.IndexerSubSys;

import org.livoniawarriors.swerve.SwerveDriveTrain;

public class ShootAndBackAuton extends SequentialCommandGroup {
    public ShootAndBackAuton(SwerveDriveTrain swerveObj, ShooterSubSys shooterObj, IndexerSubSys indexerObj) {
        double distance = 2.0; //units in meters
        double tgtVelocity = 60.0;  //units in RPM
        addCommands(
                new ShootRingAuton(shooterObj, indexerObj, tgtVelocity),
                new DriveBackCmd(swerveObj, distance)
            );
    }
}
