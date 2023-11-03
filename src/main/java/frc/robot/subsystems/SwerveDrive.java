// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SwerveModule;

public class SwerveDrive extends SubsystemBase {
  private final SwerveModule[] modules;

  public SwerveDrive() {
    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };
  }

  // Fancy factory command
  public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, boolean isOpenLoop) {
    return run(() -> {
      double forwardBack = forwardBackAxis.getAsDouble();
      double leftRight = leftRightAxis.getAsDouble();
      double rotation = rotationAxis.getAsDouble();

      // Make sure it doesnt run too slow so the motors don't go bye bye
      forwardBack = Math.abs(forwardBack) < Constants.kControls.kAxisDeadzone ? 0 : forwardBack;
      leftRight = Math.abs(leftRight) < Constants.kControls.kAxisDeadzone ? 0 : leftRight;
      rotation = Math.abs(rotation) < Constants.kControls.kAxisDeadzone ? 0 : rotation;

      forwardBack *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      leftRight *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      rotation *= Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardBack, leftRight, rotation);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states, isOpenLoop);

      SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
      for (int i = 0; i < modules.length; i++) {
        System.out.println(getModuleString(i) + modules[i].getCanCoder());
      }



      SmartDashboard.putNumber("FrontLeft - 0", modules[0].getCanCoder());
      SmartDashboard.putNumber("FrontRight - 1", modules[1].getCanCoder());
      SmartDashboard.putNumber("BackLeft - 2", modules[2].getCanCoder());
      SmartDashboard.putNumber("BackRight - 3", modules[3].getCanCoder());
    }).withName("SwerveDriveBase");
  }

  public String getModuleString(int index) {
    if(index == 0) {
      return new String("FrontLeft");
    }
    if(index == 1) {
      return new String("FrontLRight");
    }
    if(index == 2) {
      return new String("BackLeft");
    }
    if(index == 3) {
      return new String("BackRight");
    }
    return null;
  }

  private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    // Makes sure the robot doesnt create a sonic boom (normalizes the speed if the magnitude is over a certain threshold)
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[modules[i].moduleNumber], isOpenLoop);
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getState();
    }
    return currentStates;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getPosition();
    }

    return currentStates;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /*
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    for (SwerveModule module : modules) {
      builder.addStringProperty(
        String.format("Module %d", module.moduleNumber),
        () -> {
          SwerveModuleState state = module.getState();
          return String.format("%6.2fm/s %6.3fdeg", state.speedMetersPerSecond, state.angle.getDegrees());
        },
        null);

        builder.addDoubleProperty(
          String.format("Cancoder %d", module.moduleNumber),
          () -> module.getCanCoder(),
          null);

          
        builder.addDoubleProperty(
          String.format("Angle %d", module.moduleNumber),
          () -> module.getAngle().getDegrees(),
          null);
      }
    }
  }
  */
}
