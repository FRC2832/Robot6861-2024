// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autons;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.livoniawarriors.REVColorSensor;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubSys;
import frc.robot.subsystems.IndexerSubSys;
import frc.robot.subsystems.IntakeSubSys;

public class CenterNoteAuton extends SequentialCommandGroup {
    public CenterNoteAuton(ShooterSubSys shooterObj, IndexerSubSys indexerObj, IntakeSubSys intakeObj, REVColorSensor colorSensorObj) {
        addCommands(
            new ShootRingAutoCmd(shooterObj, indexerObj, Constants.AUTON_TARGET_VELOCITY),
            new ParallelCommandGroup(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center To Note")),
                new PickUpNoteAutoCmd(intakeObj, indexerObj, colorSensorObj)
            )
            // AutoBuilder.followPath(PathPlannerPath.fromPathFile("Center Note To Shoot")),
            // new ShootRingAuton(shooterObj, indexerObj, tgtVelocity)
        );
    }
}
