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

  public static class DriveConstants{
    //
    public static final double turnP = 0.01;
    public static final double turnI = 0.0;
    public static final double turnD = 0.0;

    //
    public static final double driveP = 0.01;
    public static final double driveI = 0.0;
    public static final double driveD = 0.0;

    //How many rotations it takes to the neo to rotate the module once
    public static final double turnGearing = 12.8;
  }
}
