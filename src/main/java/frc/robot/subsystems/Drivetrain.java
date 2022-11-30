// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//ALERT: This uses the SDS SwerveLib library, which is not officially supported for 2023. We will likely be switching to the WPILib Swerve Library in the future.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.MeasurementConstants.kTrackWidthMeters / 2, Constants.MeasurementConstants.kWheelBaseMeters / 2),
    new Translation2d(Constants.MeasurementConstants.kTrackWidthMeters / 2, -Constants.MeasurementConstants.kWheelBaseMeters / 2),
    new Translation2d(-Constants.MeasurementConstants.kTrackWidthMeters / 2, Constants.MeasurementConstants.kWheelBaseMeters / 2),
    new Translation2d(-Constants.MeasurementConstants.kTrackWidthMeters / 2, -Constants.MeasurementConstants.kWheelBaseMeters / 2)
  );

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));

  private AHRS m_gyro = new AHRS();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_frontLeft = new SwerveModule(
      CANConstants.kFrontLeftDriveMotorID, 
      CANConstants.kFrontLeftSteerMotorID, 
      CANConstants.kFrontLeftEncoderID, 
      Constants.DriveConstants.kFrontLeftEncoderOffset
    );
    
    m_frontRight = new SwerveModule(
      CANConstants.kFrontRightDriveMotorID, 
      CANConstants.kFrontRightSteerMotorID, 
      CANConstants.kFrontRightEncoderID, 
      Constants.DriveConstants.kFrontRightEncoderOffset
    );

    m_backLeft = new SwerveModule(
      CANConstants.kBackLeftDriveMotorID, 
      CANConstants.kBackLeftSteerMotorID, 
      CANConstants.kBackLeftEncoderID, 
      Constants.DriveConstants.kBackLeftEncoderOffset
    );
    
    m_backRight = new SwerveModule(
      CANConstants.kBackRightDriveMotorID, 
      CANConstants.kBackRightSteerMotorID, 
      CANConstants.kBackRightEncoderID, 
      Constants.DriveConstants.kBackRightEncoderOffset
    );
    
  }

  public void zeroGyro() {
    m_gyro.reset();
  }

  public Rotation2d getGyroRotation2d() {
    return m_gyro.getRotation2d();
  }

  public void updateOdometry() {
    m_odometry.update(
      getGyroRotation2d(),
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    );
  }
  
  public void setFieldPosition(Pose2d pose) {
    m_odometry.resetPosition(pose, getGyroRotation2d());
  }

  public Pose2d getFieldPosition() {
    return m_odometry.getPoseMeters();
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    var swerveModuleStates =
      m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed,
          ySpeed,
          rot,
          getGyroRotation2d()
        )
      );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }
}
