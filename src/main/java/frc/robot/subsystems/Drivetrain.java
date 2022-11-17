// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//ALERT: This uses the SDS SwerveLib library, which is not officially supported for 2023. We will likely be switching to the WPILib Swerve Library in the future.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
// import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase {

  private SwerveModule m_frontLeft;
  private SwerveModule m_frontRight;
  private SwerveModule m_backLeft;
  private SwerveModule m_backRight;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(Constants.MeasurementConstants.kTrackWidthMeters / 2, Constants.MeasurementConstants.kWheelBaseMeters / 2),
    new Translation2d(Constants.MeasurementConstants.kTrackWidthMeters / 2, -Constants.MeasurementConstants.kWheelBaseMeters / 2),
    new Translation2d(-Constants.MeasurementConstants.kTrackWidthMeters / 2, Constants.MeasurementConstants.kWheelBaseMeters / 2),
    new Translation2d(-Constants.MeasurementConstants.kTrackWidthMeters / 2, -Constants.MeasurementConstants.kWheelBaseMeters / 2)
  );

  //See periodic() for why this is commented out
  // private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0.0));

  private AHRS m_gyro = new AHRS();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    m_frontLeft = Mk4iSwerveModuleHelper.createNeo(
    Mk4iSwerveModuleHelper.GearRatio.L2,
    Constants.CANConstants.kFrontLeftDriveMotorID,
    Constants.CANConstants.kFrontLeftSteerMotorID,
    Constants.CANConstants.kFrontLeftEncoderID,
    Constants.CANConstants.kFrontLeftEncoderOffset
  );

  m_frontRight = Mk4iSwerveModuleHelper.createNeo(
    Mk4iSwerveModuleHelper.GearRatio.L2,
    Constants.CANConstants.kFrontRightDriveMotorID,
    Constants.CANConstants.kFrontRightSteerMotorID,
    Constants.CANConstants.kFrontRightEncoderID,
    Constants.CANConstants.kFrontRightEncoderOffset
  );

  m_backLeft = Mk4iSwerveModuleHelper.createNeo(
    Mk4iSwerveModuleHelper.GearRatio.L2,
    Constants.CANConstants.kBackLeftDriveMotorID,
    Constants.CANConstants.kBackLeftSteerMotorID,
    Constants.CANConstants.kBackLeftEncoderID,
    Constants.CANConstants.kBackLeftEncoderOffset
  );

  m_backRight = Mk4iSwerveModuleHelper.createNeo(
    Mk4iSwerveModuleHelper.GearRatio.L2,
    Constants.CANConstants.kBackRightDriveMotorID,
    Constants.CANConstants.kBackRightSteerMotorID,
    Constants.CANConstants.kBackRightEncoderID,
    Constants.CANConstants.kBackRightEncoderOffset
  );
  }

  public void zeroGyro() {
    m_gyro.reset();
  }

  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.set(moduleStates[0].speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.DriveConstants.kMaxVoltage, moduleStates[0].angle.getRadians());
    m_frontRight.set(moduleStates[1].speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.DriveConstants.kMaxVoltage, moduleStates[1].angle.getRadians());
    m_backLeft.set(moduleStates[2].speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.DriveConstants.kMaxVoltage, moduleStates[2].angle.getRadians());
    m_backRight.set(moduleStates[3].speedMetersPerSecond / Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.DriveConstants.kMaxVoltage, moduleStates[3].angle.getRadians());

    // Odometry is a bit broken when used with the SDS helper right now, so we're not using it yet
    // We'll use it when it's fixed
    // m_odometry.update(getGyroRotation(), moduleStates[1], moduleStates[2], moduleStates[3]);
  }
}
