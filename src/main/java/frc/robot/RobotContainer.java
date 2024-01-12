// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kControls;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  private final SwerveDrive m_driveSubsystem = new SwerveDrive();
  private final Vision m_visionSubsystem = new Vision();

  private final CommandXboxController m_driverController =
      new CommandXboxController(kControls.DRIVE_CONTROLLER_ID);

  public RobotContainer() {
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /**/
    m_driveSubsystem.setDefaultCommand(m_driveSubsystem.drive(
      () -> -Constants.kControls.X_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS)),
      () -> -Constants.kControls.Y_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(Constants.kControls.TRANSLATION_X_AXIS)),
      () -> -Constants.kControls.THETA_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(Constants.kControls.ROTATION_AXIS)),
      false,
      true
    ));
    
    //m_driveSubsystem.setDefaultCommand(m_driveSubsystem.jogTurnMotors(1 * Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND, false));

    //m_driveSubsystem.setDefaultCommand(m_driveSubsystem.CANCoderTuningCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}

