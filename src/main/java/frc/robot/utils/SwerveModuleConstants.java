// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class SwerveModuleConstants {
    public final int wheelMotorID;
    public final int turnMotorID;
    public final int canCoderID;
    public final double canCoderOffsetDegrees;

    public SwerveModuleConstants(int wheelMotorID, int turnMotorID, int canCoderID, double canCoderOffsetDegrees) {
        this.wheelMotorID = wheelMotorID;
        this.turnMotorID = turnMotorID;
        this.canCoderID = canCoderID;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
    }
}
