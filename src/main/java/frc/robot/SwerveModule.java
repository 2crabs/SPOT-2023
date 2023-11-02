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

    //needs a value 0-360
    public void turnToAngle(double targetAngle){
        turnPID.setReference((targetAngle/360.0)*12.8, CANSparkMax.ControlType.kPosition);
    }

    public void setSpeed(double speed){
        wheelPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    // Takes 2 angles and finds the distance between them
    public double angleDistance(double angle1, double angle2){
        double corrected1 = correctedAngle(angle1);
        double corrected2 = correctedAngle(angle2);
        double distance = Math.abs(corrected1-corrected2);
        if(distance < 180){
            return distance;
        } else{
            return 360-distance;
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
