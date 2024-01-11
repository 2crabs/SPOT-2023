// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private final NetworkTable m_limelightTable;

  private double targetOffsetX, targetOffsetY, targetArea, targetSkew;
  private final NetworkTableEntry m_led_entry;

  /** Creates a new Vision. */
  public Vision() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    m_led_entry = m_limelightTable.getEntry("ledMode");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetOffsetX = m_limelightTable.getEntry("tx").getDouble(0);
    targetOffsetY = m_limelightTable.getEntry("ty").getDouble(0);
    targetArea = m_limelightTable.getEntry("ta").getDouble(0);
    targetSkew = m_limelightTable.getEntry("ts").getDouble(0);
  }

  public double getTargetOffsetX() {
    return targetOffsetX;
  }
  public double getTargetOffsetY() {
    return targetOffsetY;
  }
  public double getTargetArea() {
    return targetArea;
  }
  public double getTargetSkew() {
    return targetSkew;
  }
}
