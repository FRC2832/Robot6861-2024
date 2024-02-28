// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubSys;
import frc.robot.subsystems.ShooterSubSys;

public class AmpSideOutAuton extends SequentialCommandGroup {
    public AmpSideOutAuton(ShooterSubSys shooterObj, IndexerSubSys indexerObj) {
        addCommands(
            new ShootRingAutoCmd(shooterObj, indexerObj, Constants.AUTON_TARGET_VELOCITY),
            new WaitAutoCmd(9),
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Amp Side Out"))
        );
    }
}
