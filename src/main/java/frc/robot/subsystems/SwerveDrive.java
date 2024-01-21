// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;

import com.kauailabs.navx.frc.AHRS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ModulePosition;
import frc.robot.utils.SwerveModule;

public class SwerveDrive extends SubsystemBase {
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);;
  public double targetRotation = 0.0;
  public PIDController robotRotationPID = new PIDController(25.0, 0.0, 0.0);

  SwerveDrivePoseEstimator poseEstimator;
  private final EnumMap<ModulePosition,SwerveModule> modules;

  public SwerveDrive() {
    modules = new EnumMap<>(ModulePosition.class);
    modules.put(ModulePosition.FRONT_LEFT, new SwerveModule(Constants.kSwerve.FRONT_LEFT_MODULE));
    modules.put(ModulePosition.FRONT_RIGHT, new SwerveModule(Constants.kSwerve.FRONT_RIGHT_MODULE));
    modules.put(ModulePosition.BACK_LEFT, new SwerveModule(Constants.kSwerve.BACK_LEFT_MODULE));
    modules.put(ModulePosition.BACK_RIGHT, new SwerveModule(Constants.kSwerve.BACK_RIGHT_MODULE));
    gyro.zeroYaw();
    poseEstimator = new SwerveDrivePoseEstimator(Constants.kSwerve.KINEMATICS, getGyroRotation(), getModulePositions(), Constants.kSwerve.INITIAL_POSE);

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveWithChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              DriverStation.Alliance alliance = DriverStation.getAlliance();
              return alliance == DriverStation.Alliance.Red;

            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void periodic(){
    poseEstimator.update(getGyroRotation(), getModulePositions());
  }

  /* Basic Swerve Drive Method */
  public void drive(Double forwardSpeed, Double sidewaysSpeed, Double newRotationTarget, boolean withRotation, boolean isFieldOriented) {
    targetRotation = newRotationTarget;
    double pidRotation = robotRotationPID.calculate(getGyroRotation().getRotations(), targetRotation);

    if (pidRotation>Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND){
      pidRotation = Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;
    } else if(pidRotation<-1.0*Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND){
      pidRotation = -1.0*Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;
    }

    ChassisSpeeds chassisSpeeds;
    if (withRotation){
      chassisSpeeds = new ChassisSpeeds(forwardSpeed, sidewaysSpeed, pidRotation);
    } else if (Math.abs(targetRotation-getGyroRotation().getRotations())< 1.5/360){
      chassisSpeeds = new ChassisSpeeds(forwardSpeed, sidewaysSpeed, 0.0);
    } else {
      chassisSpeeds = new ChassisSpeeds(forwardSpeed, sidewaysSpeed, 0.0);
    }

    SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(isFieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroRotation()) : chassisSpeeds);

    setModuleStates(states, false);
  }

  public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds){
    setModuleStates(Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds), false);
  }

  public ChassisSpeeds getCurrentSpeeds(){
    return Constants.kSwerve.KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose){
    poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
  }

  public Command jogTurnMotors(double speed, boolean isOpenLoop) {
    return run(() -> {
      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0, 0);
      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states, isOpenLoop);
    });
  }

  private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    // Makes sure the robot doesnt create a sonic boom (normalizes the speed if the magnitude is over a certain threshold)
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

    // iterates though the modules and gets state based on position (ModulePosition)
    modules.forEach(
            (key, value)->
                    value.setState(states[positionAsNumber(key)], isOpenLoop)
    );
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
            modules.get(ModulePosition.FRONT_LEFT).getState(),
            modules.get(ModulePosition.FRONT_RIGHT).getState(),
            modules.get(ModulePosition.BACK_LEFT).getState(),
            modules.get(ModulePosition.BACK_RIGHT).getState()
    };
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
            modules.get(ModulePosition.FRONT_LEFT).getPosition(),
            modules.get(ModulePosition.FRONT_RIGHT).getPosition(),
            modules.get(ModulePosition.BACK_LEFT).getPosition(),
            modules.get(ModulePosition.BACK_RIGHT).getPosition()
    };
  }

  // Zero Gyro
  public void zeroGyroscope() {
    gyro.calibrate();
  }

  public Rotation2d getGyroRotation() {
    return new Rotation2d(gyro.getAngle()*(Math.PI/-180.0));
  }

  /**
   * This function is needed to convert between ModulePosition and an index uses in a SwerveModuleState array
   * @param modulePosition The position of the module
   * @return The corresponding index in the list returned by toSwerveModuleStates
   */
  public int positionAsNumber(ModulePosition modulePosition){
    switch (modulePosition){
      case FRONT_LEFT: return 0;
      case FRONT_RIGHT: return 1;
      case BACK_LEFT: return 2;
      case BACK_RIGHT: return 3;
    }
    return 0;
  }
}
