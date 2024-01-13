// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ModulePosition;
import frc.robot.utils.SwerveModule;

public class SwerveDrive extends SubsystemBase {
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);;

  public static double robotDirection = 0;

  public double targetAngle = 0.0;
  public PIDController driftCorrection = new PIDController(25.0, 0.0, 0.0);

  SwerveDriveOdometry odometry;
  private final EnumMap<ModulePosition,SwerveModule> modules;

  public SwerveDrive() {
    modules = new EnumMap<>(ModulePosition.class);
    modules.put(ModulePosition.FRONT_LEFT, new SwerveModule(Constants.kSwerve.FRONT_LEFT_MODULE));
    modules.put(ModulePosition.FRONT_RIGHT, new SwerveModule(Constants.kSwerve.FRONT_RIGHT_MODULE));
    modules.put(ModulePosition.BACK_LEFT, new SwerveModule(Constants.kSwerve.BACK_LEFT_MODULE));
    modules.put(ModulePosition.BACK_RIGHT, new SwerveModule(Constants.kSwerve.BACK_RIGHT_MODULE));
    m_gyro.zeroYaw();
    odometry = new SwerveDriveOdometry(Constants.kSwerve.KINEMATICS, getGyroRotation(), getModulePositions());
  }

  public void periodic(){
    odometry.update(getGyroRotation(), getModulePositions());
  }

  // Fancy factory command
  public void drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, boolean isOpenLoop, boolean isFieldOriented) {
      SmartDashboard.putNumber("Gyro Angle", getGyroRotation().getDegrees());

      targetAngle += Math.abs(rotationAxis.getAsDouble()) < Constants.kControls.AXIS_DEADZONE ? 0 : rotationAxis.getAsDouble()/150.0;

      SmartDashboard.putNumber("targetAngle", targetAngle);
      SmartDashboard.putData(driftCorrection);

      double pidRotation = 0.0;
      if(Math.abs(getGyroRotation().getRotations()-targetAngle) > 1.5/360.0){
        pidRotation = driftCorrection.calculate(getGyroRotation().getRotations(), targetAngle);
      } else{
        driftCorrection.calculate(getGyroRotation().getRotations(), targetAngle);
      }

      SmartDashboard.putNumber("pidRotation", pidRotation);


      if (pidRotation>Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND){
        pidRotation = Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;
      } else if(pidRotation<-1.0*Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND){
        pidRotation = -1.0*Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;
      }
      

      double forwardBack = forwardBackAxis.getAsDouble();
      double leftRight = leftRightAxis.getAsDouble();
      //only stop turning if going slowly
      double rotation = pidRotation;
      if(Math.sqrt((forwardBack*forwardBack)+(leftRight*leftRight)) < 0.15){
        rotation = Math.abs(rotationAxis.getAsDouble()) < Constants.kControls.AXIS_DEADZONE ? 0.0 : pidRotation;
      }
      

      // Make sure it doesnt run too slow so the motors don't go bye bye
      forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
      leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;

      forwardBack *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      leftRight *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardBack, leftRight, rotation);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(isFieldOriented ? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroRotation()) : chassisSpeeds);

      setModuleStates(states, isOpenLoop);
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
    m_gyro.calibrate();
  }

  public Rotation2d getGyroRotation() {
    return new Rotation2d(m_gyro.getAngle()*(Math.PI/-180.0));
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
