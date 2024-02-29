// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.livoniawarriors.REVColorSensor;
import org.livoniawarriors.leds.LedSubsystem;
import org.livoniawarriors.leds.LightningFlash;
import org.livoniawarriors.leds.RainbowLeds;
import org.livoniawarriors.leds.TestLeds;
import org.livoniawarriors.odometry.Odometry;
import org.livoniawarriors.odometry.PigeonGyro;
import org.livoniawarriors.odometry.SimSwerveGyro;
import org.livoniawarriors.swerve.DriveXbox;
import org.livoniawarriors.swerve.MoveWheels;
import org.livoniawarriors.swerve.SwerveDriveSim;
import org.livoniawarriors.swerve.SwerveDriveTrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbDownCmd;
import frc.robot.commands.ClimbUpCmd;
import frc.robot.commands.ShowClimberEncoderCmd;
import frc.robot.commands.autons.PickUpNoteAutoCmd;
import frc.robot.commands.autons.ShootRingAutoCmd;
import frc.robot.commands.autons.WaitAutoCmd;
import frc.robot.commands.IntakeNoteCmd;
import frc.robot.commands.OuttakeNoteCmd;
import frc.robot.commands.PrimeShooterCmd;
import frc.robot.commands.ReverseShooterCmd;
import frc.robot.commands.RunIndexDownCmd;
import frc.robot.commands.RunIndexUpCmd;
import frc.robot.commands.RunIndexUpContinuousCmd;
import frc.robot.subsystems.ClimberSubSys;
import frc.robot.subsystems.IndexerSubSys;
import frc.robot.subsystems.IntakeSubSys;
import frc.robot.subsystems.PracticeSwerveHw;
import frc.robot.subsystems.ShooterSubSys;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private SwerveDriveTrain swerveDrive;
    private Odometry odometry;
    private LedSubsystem leds;
    private IntakeSubSys intakeSubSysObj;
    private IndexerSubSys indexerSubSysObj;
    private ShooterSubSys shooterSubSysObj;
    private REVColorSensor colorSensorObj;
    private ClimberSubSys climberSubSysObj;
    // private VisionSystem visionSystemObj;
    private UsbCamera camera;
    // TODO: Make a JoystickSubSystem
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);
        intakeSubSysObj = new IntakeSubSys();
        indexerSubSysObj = new IndexerSubSys();
        shooterSubSysObj = new ShooterSubSys();
        climberSubSysObj = new ClimberSubSys();
        colorSensorObj = new REVColorSensor(Port.kMXP); // TODO: Verify this port.
        String serNum = RobotController.getSerialNumber();
        SmartDashboard.putString("Serial Number", serNum);
        // known Rio serial numbers:
        // 031b525b = buzz
        // 03064db7 = big buzz

        // Boilerplate code to start the camera server
        camera = CameraServer.startAutomaticCapture(Constants.CAMERA_USB_PORT);
        camera.setResolution(Constants.IMAGE_WIDTH, Constants.IMAGE_HEIGHT);
        camera.setFPS(Constants.FRAMERATE);

        // CvSink cvSink = CameraServer.getVideo();

        // subsystems used in all robots
        odometry = new Odometry();
        leds = new LedSubsystem(0, 10);
        // new VisionSystem(odometry); //not making variable as we won't change this
        // subsystem

        // build the robot based on the Rio ID of the robot
        if (RobotBase.isSimulation() || (serNum.equals("031b525b")) || (serNum.equals("03064db7"))) {
            // either buzz or simulation
            swerveDrive = new SwerveDriveTrain(new SwerveDriveSim(), odometry);
            odometry.setGyroHardware(new SimSwerveGyro(swerveDrive));
        } else {
            // competition robot
            swerveDrive = new SwerveDriveTrain(new PracticeSwerveHw(), odometry);
            odometry.setGyroHardware(new PigeonGyro(50)); // TODO: Ensure this ID is correct

        }

        odometry.setSwerveDrive(swerveDrive);
        odometry.setStartingPose(new Pose2d(1.92, 2.79, new Rotation2d(0))); // TODO: Verify this starting pose is
                                                                             // correct.

        // visionSystemObj = new VisionSystem(odometry);

        // add some buttons to press for development
        SmartDashboard.putData("Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.wheelsStraight()));
        SmartDashboard.putData("Wheels Crossed", new MoveWheels(swerveDrive, MoveWheels.wheelsCrossed()));
        SmartDashboard.putData("Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.wheelsDiamond()));
        SmartDashboard.putData("Drive Wheels Straight", new MoveWheels(swerveDrive, MoveWheels.driveWheelsStraight()));
        SmartDashboard.putData("Drive Wheels Diamond", new MoveWheels(swerveDrive, MoveWheels.driveWheelsDiamond()));
        SmartDashboard.putData("Test Leds", new TestLeds(leds));
        SmartDashboard.putNumber("Climb motor encoder", climberSubSysObj.showEncoders());
        

        // Register Named Commands for PathPlanner
        NamedCommands.registerCommand("flashRed", new LightningFlash(leds, Color.kFirstRed));
        NamedCommands.registerCommand("flashBlue", new LightningFlash(leds, Color.kFirstBlue));
        NamedCommands.registerCommand("PickUpNote", new PickUpNoteAutoCmd(intakeSubSysObj, indexerSubSysObj, colorSensorObj));
        NamedCommands.registerCommand("ShootRing", new ShootRingAutoCmd(shooterSubSysObj, indexerSubSysObj, Constants.AUTON_TARGET_VELOCITY));
        NamedCommands.registerCommand("Wait", new WaitAutoCmd(8));

        // Configure the AutoBuilder
        AutoBuilder.configureHolonomic(
                odometry::getPose, // Robot pose supplier
                odometry::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                swerveDrive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                swerveDrive::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(3, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.25, 0.0, 0.0), // Rotation PID constants
                        swerveDrive.getMaxSpeed(), // Max module speed, in m/s
                        swerveDrive.getDriveBaseRadius(), // Drive base radius in meters. Distance from robot center to
                                                          // furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                odometry::shouldFlipAlliance, // shouldFlipPath Supplier that determines if paths should be flipped to
                                              // the other side of the field. This will maintain a global blue alliance
                                              // origin.
                swerveDrive // Reference to this subsystem to set requirements
        );

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Do Nothing", Commands.none());
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    public void configureBindings() {
        // setup default commands that are used for driving
        swerveDrive.setDefaultCommand(new DriveXbox(swerveDrive, driverController));
        leds.setDefaultCommand(new RainbowLeds(leds));
        //ClimberSubSys.setDefaultCommand(new ShowClimberEncoderCmd(climberSubSysObj));
        

        // setup button bindings
        Trigger operatorLeftTrigger = operatorController.leftTrigger();
        Trigger operatorRightTrigger = operatorController.rightTrigger();
        Trigger operatorLeftBumper = operatorController.leftBumper();
        Trigger operatorRightBumper = operatorController.rightBumper();
        Trigger operatorAButton = operatorController.a();
        Trigger operatorYButton = operatorController.y();
        Trigger operatorDPadDown = operatorController.povDown();
        Trigger operatorDPadUp = operatorController.povUp();

        Trigger driverRightTrigger = driverController.rightTrigger();
        Trigger driverXButton = driverController.x();


        ParallelCommandGroup intakeGroup = new ParallelCommandGroup(
                new IntakeNoteCmd(intakeSubSysObj, colorSensorObj, driverController, operatorController),
                new RunIndexUpCmd(indexerSubSysObj, colorSensorObj));
        ParallelCommandGroup outtakeGroup = new ParallelCommandGroup(new OuttakeNoteCmd(intakeSubSysObj),
                new RunIndexDownCmd(indexerSubSysObj));

        // operatorRightTrigger.whileTrue(new IntakeNoteCmd(intakeSubSysObj));
        operatorLeftTrigger.whileTrue(outtakeGroup);
        operatorRightTrigger.whileTrue(intakeGroup);
        operatorLeftBumper.whileTrue(new RunIndexDownCmd(indexerSubSysObj));
        operatorRightBumper.whileTrue(new RunIndexUpCmd(indexerSubSysObj, colorSensorObj));
        operatorAButton.whileTrue(new PrimeShooterCmd(shooterSubSysObj));
        operatorYButton.whileTrue(new ReverseShooterCmd(shooterSubSysObj));
        operatorDPadDown.whileTrue(new ClimbDownCmd(climberSubSysObj));
        operatorDPadUp.whileTrue(new ClimbUpCmd(climberSubSysObj));

        driverRightTrigger.whileTrue(new RunIndexUpContinuousCmd(indexerSubSysObj));
        driverXButton.whileTrue(new RunIndexDownCmd(indexerSubSysObj));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
