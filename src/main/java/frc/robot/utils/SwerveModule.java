package frc.robot.utils;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {

  private final CANSparkMax wheelMotor;
  private final RelativeEncoder wheelEncoder;
  private final SparkMaxPIDController wheelPID;
  private final SimpleMotorFeedforward wheelFeedforward;

  private final CANSparkMax turnMotor;
  private final RelativeEncoder turnEncoder;
  private final SparkMaxPIDController turnPID;
  
  private final CANCoder turnCANCoder;
  private final double CANCoderOffsetDegrees;

  private double lastAngle;

  public SwerveModule(SwerveModuleConstants moduleConstants) {
    
    turnMotor = new CANSparkMax(moduleConstants.turnMotorID, MotorType.kBrushless);
    wheelMotor = new CANSparkMax(moduleConstants.wheelMotorID, MotorType.kBrushless);
    
    wheelPID = wheelMotor.getPIDController();
    turnPID = turnMotor.getPIDController();

    wheelEncoder = wheelMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    turnCANCoder = new CANCoder(moduleConstants.canCoderID);
    CANCoderOffsetDegrees = moduleConstants.canCoderOffsetDegrees;

    wheelFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);

    configureDevices();

    lastAngle = getState().angle.getRadians();
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    // this does everything in the efficientAngle function
    state = SwerveModuleState.optimize(state, getState().angle);

    if (isOpenLoop) {
      double speed = state.speedMetersPerSecond / Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      wheelPID.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
    } else {
      wheelPID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, wheelFeedforward.calculate(state.speedMetersPerSecond));
    }

    double angle = state.angle.getRadians();
    //if (Math.abs(state.speedMetersPerSecond) <= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND) {
    //  angle = lastAngle;
    //}

    turnPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    double velocity = wheelEncoder.getVelocity();
    Rotation2d angle = new Rotation2d(turnEncoder.getPosition());
    return new SwerveModuleState(velocity, angle);
  }

  public double getTurnCANCoder() {
    return turnCANCoder.getAbsolutePosition();
  }

  public Rotation2d getAngle() {
    return new Rotation2d(turnEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    double distance = wheelEncoder.getPosition();
    Rotation2d rot = new Rotation2d(turnEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }
  
  private void configureDevices() {
    // Drive motor configuration.
    wheelMotor.restoreFactoryDefaults();
    wheelMotor.setInverted(Constants.kSwerve.DRIVE_MOTOR_REVERSED);
    wheelMotor.setIdleMode(Constants.kSwerve.DRIVE_IDLE_MODE);
    wheelMotor.setOpenLoopRampRate(Constants.kSwerve.OPEN_LOOP_RAMP);
    wheelMotor.setClosedLoopRampRate(Constants.kSwerve.CLOSED_LOOP_RAMP);
    wheelMotor.setSmartCurrentLimit(Constants.kSwerve.DRIVE_CURRENT_LIMIT);
 
    wheelPID.setP(Constants.kSwerve.DRIVE_KP);
    wheelPID.setI(Constants.kSwerve.DRIVE_KI);
    wheelPID.setD(Constants.kSwerve.DRIVE_KD);
    wheelPID.setFF(Constants.kSwerve.DRIVE_KF);
 
    wheelEncoder.setPositionConversionFactor(Constants.kSwerve.DRIVE_ROTATIONS_TO_METERS);
    wheelEncoder.setVelocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
    wheelEncoder.setPosition(0);

    // Angle motor configuration.
    turnMotor.restoreFactoryDefaults();
    turnMotor.setInverted(Constants.kSwerve.ANGLE_MOTOR_REVERSED);
    turnMotor.setIdleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
    turnMotor.setSmartCurrentLimit(Constants.kSwerve.ANGLE_CURRENT_LIMIT);

    turnPID.setP(Constants.kSwerve.ANGLE_KP);
    turnPID.setI(Constants.kSwerve.ANGLE_KI);
    turnPID.setD(Constants.kSwerve.ANGLE_KD);
    turnPID.setFF(Constants.kSwerve.ANGLE_KF);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
    turnPID.setPositionPIDWrappingMinInput(0);

    // PIDS
    turnPID.setReference(0, ControlType.kPosition);
    wheelPID.setReference(0, ControlType.kDutyCycle);

    // CanCoder configuration.
    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.sensorDirection = Constants.kSwerve.CAN_CODER_REVERSED;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
    
    turnCANCoder.configFactoryDefault();
    turnCANCoder.configAllSettings(canCoderConfiguration);

    turnEncoder.setPositionConversionFactor(Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
    turnEncoder.setVelocityConversionFactor(Constants.kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);
    turnEncoder.setPosition(Math.toRadians(turnCANCoder.getAbsolutePosition()+ CANCoderOffsetDegrees));
  }
}