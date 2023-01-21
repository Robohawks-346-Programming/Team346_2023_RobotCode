// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // Pneumatic Channels
  public static final int GRABBER_OUT_PNEUMATIC_ID = 1;
  public static final int GRABBER_IN_PNEUMATIC_ID  = 2;

  // Motor IDs
  public static final int ARM_MOTOR_ID        = 3;

  // Motor Speed
  public static final double ARM_MOTOR_SPEED  = 0.5;

  // Motor Revolutions
  public static final double ARM_REV       = 1;
  public static final double GROUND_ARM_POSITION = 1;
  public static final double FIRST_PEG_ARM_POSITION = 1;
  public static final double SECOND_PEG_ARM_POSITION = 1;
  public static final double FIRST_SHELF_ARM_POSITION = 1;
  public static final double SECOND_SHELF_ARM_POSITION = 1;
}
