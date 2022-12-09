// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_steerMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_steerIntegratedEncoder;
  private final CANCoder m_steerEncoder;

  private final SparkMaxPIDController m_steerPIDController;

  private final double m_steerEncoderOffset;

  private SwerveModulePosition m_modulePosition;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorID, int steerMotorID, int steerEncoderID, double steerEncoderOffset) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_steerIntegratedEncoder = m_steerMotor.getEncoder();
    m_steerEncoder = new CANCoder(steerEncoderID); // TODO: Configure the CANCoder to not reset on power cycle, unsigned angle return

    motorInitialize(m_driveMotor);
    motorInitialize(m_steerMotor);
    
    m_driveMotor.setSmartCurrentLimit(Constants.DriveConstants.kDriveCurrentLimit);

    m_steerMotor.setSmartCurrentLimit(Constants.DriveConstants.kSteerCurrentLimit);
      
    double positionConversionFactor = Math.PI * Constants.MeasurementConstants.kWheelDiameterMeters * Constants.DriveConstants.kDriveReduction; 
    m_driveEncoder.setPositionConversionFactor(positionConversionFactor); // Gives meters
    m_driveEncoder.setVelocityConversionFactor(positionConversionFactor / 60.0); // Gives meters per second

    positionConversionFactor = 360 * Constants.DriveConstants.kSteerReduction; 
    m_steerIntegratedEncoder.setPositionConversionFactor(positionConversionFactor); // Gives degrees
    m_steerIntegratedEncoder.setVelocityConversionFactor(positionConversionFactor / 60.0); // Gives degrees per second
    m_steerIntegratedEncoder.setPosition(m_steerEncoder.getAbsolutePosition());

    m_steerPIDController = m_steerMotor.getPIDController();

    m_steerEncoderOffset = steerEncoderOffset;

    // TODO: Set PID constants
    m_steerPIDController.setP(Constants.DriveConstants.kSteerP);
    m_steerPIDController.setI(Constants.DriveConstants.kSteerI);
    m_steerPIDController.setD(Constants.DriveConstants.kSteerD);

    m_modulePosition = new SwerveModulePosition();
  }

  private void motorInitialize(CANSparkMax motor){
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 100);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 20);
    motor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 20);
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.enableVoltageCompensation(Constants.DriveConstants.kMaxVoltage);
    motor.setInverted(true);
  }

  public void reset(){
    m_steerIntegratedEncoder.setPosition(m_steerEncoder.getAbsolutePosition());
  }

  public void stop(){
    m_driveMotor.set(0);
    m_steerMotor.set(0);
  }

  public void updatePosition(){
    m_modulePosition.angle = getSteerAngle();
    m_modulePosition.distanceMeters = getDriveDistance();
  }

  public SwerveModulePosition getPosition(){
    updatePosition();
    return m_modulePosition;
  }

  public Rotation2d getSteerAngle(){
    double angle = m_steerIntegratedEncoder.getPosition() - m_steerEncoderOffset;
    return Rotation2d.fromDegrees(angle); 
  }

  public double getDriveDistance(){
    return m_driveEncoder.getPosition();
  }

  public SwerveModuleState getModuleState(){
    return new SwerveModuleState(m_driveMotor.getEncoder().getVelocity(), getSteerAngle());
  }

  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getSteerAngle());
    m_driveMotor.set(state.speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    m_steerPIDController.setReference(state.angle.getDegrees() + m_steerEncoderOffset, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
