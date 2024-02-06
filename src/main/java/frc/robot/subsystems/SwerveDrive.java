// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
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
  }

  public void periodic(){
    poseEstimator.update(getGyroRotation(), getModulePositions());
  }

  /** Drive command that allows the usage of a PID controller to reach a rotation
   * @param forwardSpeed forward speed in m/s
   * @param sidewaysSpeed sideways speed in m/s
   * @param  newRotationTarget what rotation the robot should go to. This number is continuous and needs to be based on the current targetRotation
   * @param withRotation determines if robot will use a PID controller to reach the newRotation Target. If you want to use the PID for rotation but don't want to set a new rotation just feed in swervedrive.targetRotation
   * @param isFieldOriented whether to use field relative for translation
   */
  public void drive(Double forwardSpeed, Double sidewaysSpeed, Double newRotationTarget, boolean withRotation, boolean isFieldOriented) {
    targetRotation = newRotationTarget;
    double pidRotation = robotRotationPID.calculate(getGyroRotation().getRotations(), newRotationTarget);

    //Makes sure the output of PID controller stays within max angualr speed
    if (pidRotation>Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND){
      pidRotation = Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;
    } else if(pidRotation<-1.0*Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND){
      pidRotation = -1.0*Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;
    }

    ChassisSpeeds chassisSpeeds;
    // if using rotation and the robot is not at the target rotation
    if (withRotation && !(Math.abs(targetRotation-getGyroRotation().getRotations())< 1.5/360)){
      chassisSpeeds = new ChassisSpeeds(forwardSpeed, sidewaysSpeed, pidRotation);
    } else {
      chassisSpeeds = new ChassisSpeeds(forwardSpeed, sidewaysSpeed, 0.0);
    }

    SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(isFieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroRotation()) : chassisSpeeds);

    setModuleStates(states, false);
  }

  /** Drive command without PID controller for rotation
   *
   * @param forwardSpeed
   * @param sidewaysSpeed
   * @param rotation
   * @param isFieldOriented
   */
  public void basicDrive(Double forwardSpeed, Double sidewaysSpeed, double rotation, boolean isFieldOriented) {
    targetRotation = getGyroRotation().getRotations();
    double pidRotation = robotRotationPID.calculate(getGyroRotation().getRotations(), targetRotation);

    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(forwardSpeed, sidewaysSpeed, rotation);
    SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(isFieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroRotation()) : chassisSpeeds);

    setModuleStates(states, false);
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
