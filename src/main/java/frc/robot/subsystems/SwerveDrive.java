// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.ModulePosition;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.SwerveModuleConstants;

public class SwerveDrive extends SubsystemBase {
  private final EnumMap<ModulePosition,SwerveModule> modules;

  public SwerveDrive() {
    modules = new EnumMap<>(ModulePosition.class);
    modules.put(ModulePosition.FRONT_LEFT, new SwerveModule(Constants.kSwerve.FRONT_LEFT_MODULE));
    modules.put(ModulePosition.FRONT_RIGHT, new SwerveModule(Constants.kSwerve.FRONT_RIGHT_MODULE));
    modules.put(ModulePosition.BACK_LEFT, new SwerveModule(Constants.kSwerve.BACK_LEFT_MODULE));
    modules.put(ModulePosition.BACK_RIGHT, new SwerveModule(Constants.kSwerve.BACK_RIGHT_MODULE));
  }

  // Fancy factory command
  public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, boolean isOpenLoop) {
    return run(() -> {
      double forwardBack = forwardBackAxis.getAsDouble();
      double leftRight = leftRightAxis.getAsDouble();
      double rotation = rotationAxis.getAsDouble();

      // Make sure it doesnt run too slow so the motors don't go bye bye
      forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
      leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;
      rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;

      forwardBack *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      leftRight *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      rotation *= Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardBack, leftRight, rotation);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states, isOpenLoop);
    }).withName("SwerveDriveBase");
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
