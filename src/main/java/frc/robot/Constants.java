// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // CANID Constants
    public static final int INTAKE_MOTOR_CAN_ID = 2;

    // Intake Motor Values
    public static final double INTAKE_MOTOR_PCT = 30.0;
    public static final double OUTTAKE_MOTOR_PCT = -20.0;
    public static final int INTAKE_MOTOR_SMART_CURRENT_LIMIT = 5;

    private Constants() {
        // No-op. This is a utility class. Keep the constructor private to prevent instantiation.
    }
}