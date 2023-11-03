package frc.robot.utils;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;
import edu.wpi.first.math.Pair;
import frc.robot.Constants;
import frc.robot.Constants.kSwerve;;

public class SwerveModule {
    public CANSparkMax turnMotor;
    public CANSparkMax wheelMotor;

    public CANCoder absoluteAngleEncoder;
    public RelativeEncoder turnMotorEncoder;
    public RelativeEncoder wheelEncoder;

    public SparkMaxPIDController turnPID;
    public SparkMaxPIDController wheelPID;

    //used to determine if the wheel should drive backwards or forwards
    public boolean isWheelReversed;

    public SwerveModule (int turnID, int wheelID, int encoderID) {
        turnMotor = new CANSparkMax(turnID, CANSparkMaxLowLevel.MotorType.kBrushless);
        wheelMotor = new CANSparkMax(wheelID, CANSparkMaxLowLevel.MotorType.kBrushless);
        turnMotor.setSmartCurrentLimit(Constants.kSwerve.kAngleCurrentLimit);
        wheelMotor.setSmartCurrentLimit(Constants.kSwerve.kDriveCurrentLimit);

        turnPID = turnMotor.getPIDController();
        wheelPID = wheelMotor.getPIDController();
        configurePID();

        absoluteAngleEncoder = new CANCoder(encoderID);

        turnMotorEncoder = turnMotor.getEncoder();
        turnMotorEncoder.setPosition(0);
        wheelEncoder = wheelMotor.getEncoder();
        //can set pid values here
    }

    public void configurePID(){
        
    }

    

    //needs a value 0-360 as target angle
    public void turnToAngle(double targetAngle){
        //turns motor rotations into module degrees
        double currentRotation = (turnMotorEncoder.getPosition()/12.8)*360.0;
        double closestAngle = efficientAngle(targetAngle, currentRotation);
        turnPID.setReference((closestAngle/360.0)*12.8, CANSparkMax.ControlType.kPosition);
    }


    public void setSpeed(double speed){
        if (isWheelReversed) {
            wheelPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        } else {
            wheelPID.setReference(-speed, CANSparkMax.ControlType.kVelocity);
        }
    }

    /**
     * Finds the closest angle to currentVal that makes it equal to targetAngle or the reflection of the target angle
     * @param targetAngle Angle to go (0 to 360)
     */
    public double efficientAngle(double targetAngle, double currentVal){
        double currentAngle = correctedAngle(currentVal);
        double reflectedTargetAngle = (targetAngle+180.0)%360.0;
        double offset;
        if(Math.abs(angleOffset(currentAngle, targetAngle))<90){
            isWheelReversed = false;
            offset = angleOffset(currentAngle, targetAngle);
        } else {
            isWheelReversed = true;
            offset = angleOffset(currentAngle, reflectedTargetAngle);
        }
        return currentVal+offset;
    }

    // finds angle needed to change start to end
    public double angleOffset(double start, double end){
        double correctedStart = correctedAngle(start);
        double correctedEnd = correctedAngle(end);
        double distance = Math.abs(correctedStart-correctedEnd);
        if(distance < 180){
            return correctedEnd-correctedStart;
        } else{
            if(end>start){
                return -(360-distance);
            } else{
                return 360-distance;
            }
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
