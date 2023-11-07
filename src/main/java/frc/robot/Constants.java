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
    public static final double AXIS_DEADZONE = 0.1;

    public static final int DRIVE_CONTROLLER_ID = 0;

    public static final int TRANSLATION_X_AXIS = XboxController.Axis.kLeftX.value;
    public static final int TRANSLATION_Y_AXIS = XboxController.Axis.kLeftY.value;
    public static final int ROTATION_AXIS = XboxController.Axis.kRightX.value;

    // Prevent from acclerating/decclerating to quick
    public static final SlewRateLimiter X_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter Y_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter THETA_DRIVE_LIMITER = new SlewRateLimiter(4);
  }

  /** All swerve constants. */
  public static class kSwerve {
    /** Constants that apply to the whole drive train. */
    public static final double TRACK_WIDTH = Units.inchesToMeters(19.5); // Width of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_BASE = Units.inchesToMeters(19.5); // Length of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    );

    public static final double DRIVE_GEAR_RATIO = 6.75; // 6.75:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFRENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = 12.8; // 12.8:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int ANGLE_CURRENT_LIMIT = 25;

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
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 2.0;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 4.0;

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_REVERSED = true;
    public static final boolean ANGLE_MOTOR_REVERSED = false;
    public static final boolean CAN_CODER_REVERSED = false;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

    /** 
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */

     //FrontLeft
    public static final SwerveModuleConstants FRONT_LEFT_MODULE = new SwerveModuleConstants(
      4,
      5,
      0,
      //203.115234
      -352.97
    );

    //FrontRight
    public static final SwerveModuleConstants FRONT_RIGHT_MODULE = new SwerveModuleConstants(
      6,
      7,
      3,
      //191.074219  
      -344.53
    );

    //BackLeft
    public static final SwerveModuleConstants BACK_LEFT_MODULE = new SwerveModuleConstants(
      10,
      11,
      2,
      //203.906250
      -7.03
    );

    //BackRight
    public static final SwerveModuleConstants BACK_RIGHT_MODULE = new SwerveModuleConstants(
      8,
      9,
      1,
      //155.214844
      -114.7
    );
  }
}
