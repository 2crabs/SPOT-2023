// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class FollowCurrentTarget extends CommandBase {
  private final Vision m_visionSubsystem;
  private final SwerveDrive m_driveSubsystem;

  private double m_forward;

  /** Creates a new FollowCurrentTarget. */
  public FollowCurrentTarget(Vision visionSubsystem, SwerveDrive driveSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_driveSubsystem = driveSubsystem;
    addRequirements(visionSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("FollowingApriltag", true);

    DoubleSupplier targetXOffset = () -> -(m_visionSubsystem.getTargetOffsetX()/54);
    double targetArea = m_visionSubsystem.getTargetArea();

    m_forward = 0;
    DoubleSupplier forwardAxis = () -> m_forward;

    if(targetArea <= 2.1) {
      m_forward = 0.05;
    }

    m_driveSubsystem.drive(forwardAxis, () -> 0, targetXOffset, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("FollowingApriltag", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
