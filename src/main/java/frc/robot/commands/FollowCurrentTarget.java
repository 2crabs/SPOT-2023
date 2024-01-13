// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class FollowCurrentTarget extends CommandBase {
  private final Vision m_visionSubsystem;
  private final SwerveDrive m_driveSubsystem;

  private double m_forward;
  private double m_turn;

  private DoubleSupplier m_yAxis;
  private DoubleSupplier m_xAxis;

  /** Creates a new FollowCurrentTarget. */
  public FollowCurrentTarget(Vision visionSubsystem, SwerveDrive driveSubsystem, DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis) {
    m_visionSubsystem = visionSubsystem;
    m_driveSubsystem = driveSubsystem;

    m_yAxis = forwardBackAxis;
    m_xAxis = leftRightAxis;

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

    double turnA = Math.abs(m_visionSubsystem.getTargetOffsetX()/54);
    double turnB = m_visionSubsystem.getTargetOffsetX()/54;
    m_turn = Math.pow(3.0, turnA);
    m_turn = (m_turn-1) * 0.89;
    m_turn = -Math.copySign(m_turn, turnB);
    DoubleSupplier targetXOffset = () -> m_turn;
    double targetArea = m_visionSubsystem.getTargetArea();

    m_forward = 0;
    DoubleSupplier forwardAxis = () -> m_forward;

    if(targetArea <= 2.1 && targetArea != 0.0) {
      //m_forward = -0.2 * (2.1 - targetArea);
    }

    m_driveSubsystem.drive(m_yAxis, m_xAxis, targetXOffset, false, true);
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
