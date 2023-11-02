package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;

public class SwerveModule {
    public CANSparkMax turnMotor;
    public CANSparkMax wheelMotor;

    public CANCoder absoluteAngleEncoder;
    public RelativeEncoder turnMotorEncoder;
    public RelativeEncoder wheelEncoder;

    public SparkMaxPIDController turnPID;
    public SparkMaxPIDController wheelPID;

    public boolean isAngleReversed;

    public SwerveModule (int turnID, int wheelID, int encoderID) {
        turnMotor = new CANSparkMax(turnID, CANSparkMaxLowLevel.MotorType.kBrushless);
        wheelMotor = new CANSparkMax(wheelID, CANSparkMaxLowLevel.MotorType.kBrushless);

        absoluteAngleEncoder = new CANCoder(encoderID);

        turnMotorEncoder = turnMotor.getEncoder();
        turnMotorEncoder.setPosition(0);
        wheelEncoder = wheelMotor.getEncoder();
        //can set pid values here
    }

    //needs a value 0-360 as target angle
    public void turnToAngle(double targetAngle, double encoderVal){
        double currentAngle = correctedAngle(encoderVal);
        double reflectedTargetAngle = (targetAngle+180.0)%360.0;
        double offset;
        if(Math.abs(angleOffset(targetAngle, currentAngle))<90){
            offset = angleOffset(encoderVal, targetAngle);
        } else {
            offset = angleOffset(encoderVal, reflectedTargetAngle);
        }

        turnPID.setReference((targetAngle/360.0)*12.8, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeed(double speed){
        wheelPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    // finds number need to change start to end
    public double angleOffset(double start, double end){
        double correctedStart = correctedAngle(start);
        double correctedEnd = correctedAngle(end);
        double distance = Math.abs(correctedStart-correctedEnd);
        if(distance < 180){
            return correctedEnd-correctedStart;
        } else{
            return -(360-distance);
        }
    }

    // Takes angle and turns it into equivalent angle in range 0-360
    public double correctedAngle(double angle){
        if(angle%360.0 < 0.0){
            return angle%360.0 + 360.0;
        } else {
            return angle%360.0;
        }
    }
}
