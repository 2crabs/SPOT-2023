package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class SwerveModule {
    public CANSparkMax turnMotor;
    public CANSparkMax wheelMotor;

    public CANCoder angleEncoder;
    public RelativeEncoder turnMotorEncoder;
    public RelativeEncoder wheelEncoder;

    public SparkMaxPIDController turnPID;
    public SparkMaxPIDController wheelPID;

    public SwerveModule (int turnID, int wheelID, int encoderID) {
        turnMotor = new CANSparkMax(turnID, CANSparkMaxLowLevel.MotorType.kBrushless);
        wheelMotor = new CANSparkMax(wheelID, CANSparkMaxLowLevel.MotorType.kBrushless);
        turnMotorEncoder = turnMotor.getEncoder();
        turnMotorEncoder.setPosition(0);
        wheelEncoder = wheelMotor.getEncoder();
        //can set pid values here
    }

    public void turnToAngle(double angle){
        turnPID.setReference(angle*12.8, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeed(double speed){
        wheelPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }
}
