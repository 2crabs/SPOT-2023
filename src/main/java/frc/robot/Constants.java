// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class kControls {
    public static final double kAxisDeadzone = 0.1;

    public static final int kDriveControllerID = 0;

    public static final int kTranslationXAxis = XboxController.Axis.kLeftX.value;
    public static final int kTranslationYAxis = XboxController.Axis.kLeftY.value;
    public static final int kRotationAxis = XboxController.Axis.kRightX.value;

    // Prevent from acclerating/decclerating to quick
    public static final SlewRateLimiter X_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter Y_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter THETA_DRIVE_LIMITER = new SlewRateLimiter(4);
  }

  /** All swerve constants. */
  public static class kSwerve {
    /** Constants that apply to the whole drive train. */
    public static final double kTrackWidth = Units.inchesToMeters(19.5); // Width of the drivetrain measured from the middle of the wheels.
    public static final double kWheelBase = Units.inchesToMeters(19.5); // Length of the drivetrain measured from the middle of the wheels.
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumfrence = kWheelDiameter * Math.PI;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
      new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
    );

    public static final double kDriveGearRatio = 6.75 / 1.0; // 6.75:1
    public static final double kDriveRotationsToMeters = kWheelCircumfrence / kDriveGearRatio;
    public static final double kDriveRpmToMetersPerSecond = kDriveRotationsToMeters / 60.0;
    public static final double kAngleGearRatio = 12.8 / 1.0; // 12.8:1
    public static final double kAngleRotationsToRadians = (Math.PI * 2) / kAngleGearRatio;
    public static final double kAngleRpmToRadiansPerSecond = kDriveRotationsToMeters / 60.0;

    /** Speed ramp. */
    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    /** Current limiting. */
    public static final int kDriveCurrentLimit = 35;
    public static final int kAngleCurrentLimit = 25;

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /** Drive motor characterization. */
    public static final double DRIVE_KS = 0.11937;
    public static final double DRIVE_KV = 2.6335;
    public static final double DRIVE_KA = 0.46034;

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 1.5;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;
    
    /** Swerve constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3.0;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 4.0;

    /** Inversions. */
    public static final boolean kDriveMotorReversed = true;
    public static final boolean kAngleMotorReversed = false;
    public static final boolean kCanCoderReversed = false;

    /** Idle modes. */
    public static final IdleMode kDriveIdleMode = IdleMode.kBrake;
    public static final IdleMode kAngleIdleMode = IdleMode.kCoast;

    /** 
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */

     //FrontLeft
    public static final SwerveModuleConstants MOD_0_Constants = new SwerveModuleConstants(
      4,
      5,
      0,
      //203.115234
      352.97
    );

    //FrontRight
    public static final SwerveModuleConstants MOD_1_Constants = new SwerveModuleConstants(
      6,
      7,
      3,
      //191.074219  
      344.53
    );

    //BackLeft
    public static final SwerveModuleConstants MOD_2_Constants = new SwerveModuleConstants(
      10,
      11,
      2,
      //203.906250
      7.03
    );

    //BackRight
    public static final SwerveModuleConstants MOD_3_Constants = new SwerveModuleConstants(
      8,
      9,
      1,
      //155.214844
      114.7
    );
  }
}
