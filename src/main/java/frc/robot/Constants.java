// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // CANID Constants
    public static final int INTAKE_MOTOR_CAN_ID = 2;
    public static final int INDEX_MOTOR_CAN_ID = 1;

    // Intake Motor Values
    public static final int INTAKE_MOTOR_SMART_CURRENT_LIMIT = 10; // NEO 550 - TODO: increase once development work
    public static final int INTAKE_MOTOR_SECONDARY_CURRENT_LIMIT = 20; // complete
    public static final double INTAKE_MOTOR_PCT = 15.0; // TODO: increase these values. Maybe 70.0
    public static final double OUTTAKE_MOTOR_PCT = -20.0; // TODO: confirm these values

    // Indexer Motor Values
    public static final int INDEX_MOTOR_SMART_CURRENT_LIMIT = 10; // NEO 550 - TODO: increase once development work
    public static final int INDEX_MOTOR_SECONDARY_CURRENT_LIMIT = 20; // complete
    public static final double UPINDEX_MOTOR_PCT = 95.0; // TODO: confirm these values
    public static final double DOWNINDEX_MOTOR_PCT = -20.0; // TODO: confirm these values

    // Shooter Motor Values
    public static final int FR_SHOOTER_MOTOR_CAN_ID = 4;
    public static final int FL_SHOOTER_MOTOR_CAN_ID = 3;
    public static final int FR_SHOOTER_MOTOR_SMART_CURRENT_LIMIT = 20;
    public static final int FL_SHOOTER_MOTOR_SMART_CURRENT_LIMIT = 20;
    public static final double FR_SHOOTER_MOTOR_PCT = 10.0; // TODO: confirm these values
    public static final double FL_SHOOTER_MOTOR_PCT = -10.0; // TODO: confirm these values
    public static final double FR_SHOOTER_MOTOR_REVERSE_PCT = -2.0;
    public static final double FL_SHOOTER_MOTOR_REVERSE_PCT = 2.0;
    public static final double AUTON_TARGET_VELOCITY = 80.0;  //units in RPM

    // climber motor values NEO
    public static final int CLIMBER_MOTOR_CAN_ID = 5;
    public static final int CLIMBER_MOTOR_SMART_CURRENT_LIMIT = 50; // TODO: confirm this value
    public static final int CLIMBER_MOTOR_SECONDARY_CURRENT_LIMIT = 60; // TODO: confirm this value
    public static final int UPCLIMB_MOTOR_PCT = 20; // TODO: confirm this value
    public static final int DOWNCLIMB_MOTOR_PCT = -70; // TODO: Confirm this value

    // Swerve current limits
    public static final int DRIVE_MOTOR_PRIMARY_CURRENT_LIMIT = 40;
    public static final int DRIVE_MOTOR_SECONDARY_CURRENT_LIMIT = 60;
    public static final int TURN_MOTOR_PRIMARY_CURRENT_LIMIT = 40;
    public static final int TURN_MOTOR_SECONDARY_CURRENT_LIMIT = 60;

    // Drive mode factors
    public static final double TURTLE_MODE = 0.5;
    public static final double SNAIL_MODE = 0.25;

    // Vision system values
    public static final int CAMERA_USB_PORT = 0;
    public static final int IMAGE_WIDTH = 640;
    public static final int IMAGE_HEIGHT = 480;
    public static final int FRAMERATE = 20;

    private Constants() {
        // No-op. This is a utility class. Keep the constructor private to prevent
        // instantiation.
    }
}