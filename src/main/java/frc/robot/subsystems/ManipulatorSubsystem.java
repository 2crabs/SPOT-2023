// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ManipulatorSubsystem extends SubsystemBase {
  TalonSRX intakeAngleMotor = new TalonSRX(0);

  CANSparkMax CHANGEME = new CANSparkMax(0, null);

  /** Creates a new ManipulatorSubsystem. */
  public ManipulatorSubsystem() {
    configureHardware();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeMotorAngle(double angle) {
    
  }

  private void configureHardware() {
    // Just configure it or something
  }
}
