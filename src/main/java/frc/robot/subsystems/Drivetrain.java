// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//ALERT: This uses the SDS SwerveLib library, which is not officially supported for 2023. We will likely be switching to the WPILib Swerve Library in the future.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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

  private SwerveDriveOdometry m_odometry;

  private AHRS m_gyro = new AHRS();

  private PIDController xController;
  private PIDController yController;
  private PIDController thetaController;

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
    
    m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroRotation2d(), getModulePositions());
  }

  public void resetModules() { // Call if modules are not in the correct position
    m_frontLeft.reset();
    m_frontRight.reset();
    m_backLeft.reset();
    m_backRight.reset();
  }

  public double getOdoYaw() {
    var pos = -m_odometry.getPoseMeters().getRotation().getDegrees() % 360;
    return pos < -180 ? pos + 360 : pos;
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
      getModulePositions()
    );
  }

  public SwerveDriveKinematics getKinematics(){
    return m_kinematics;
  }

  public void zeroOdometry() {
    m_odometry.resetPosition(getGyroRotation2d(), getModulePositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[]{
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }
  
  public void setFieldPosition(Pose2d pose) {
    m_odometry.resetPosition(getGyroRotation2d(), getModulePositions(), pose);
  }

  public Pose2d getFieldPosition() {
    return m_odometry.getPoseMeters();
  }

  public void stop(){
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    SwerveModuleState[] swerveModuleStates =
      m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeed,
          ySpeed,
          rot,
          m_odometry.getPoseMeters().getRotation()
        )
      );
    
    setModuleStates(swerveModuleStates);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
      m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_odometry.getPoseMeters().getRotation()) : new ChassisSpeeds(xSpeed, ySpeed, rot)
      );
    
    setModuleStates(swerveModuleStates);
  }

  public void printModuleAbsoluteAngles() {
    System.out.println("Front Left: " + m_frontLeft.getAbsoluteAngle());
    System.out.println("Front Right: " + m_frontRight.getAbsoluteAngle());
    System.out.println("Back Left: " + m_backLeft.getAbsoluteAngle());
    System.out.println("Back Right: " + m_backRight.getAbsoluteAngle());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory) {
    xController = new PIDController(Constants.DriveConstants.kDriveP, 0, 0);
    yController = new PIDController(Constants.DriveConstants.kDriveP, 0, 0);
    thetaController = new PIDController(Constants.DriveConstants.kTurnP, 0, 0); //Kp value, Ki=0, Kd=0
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    setFieldPosition(trajectory.getInitialHolonomicPose());

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
      trajectory,
      this::getFieldPosition,
      m_kinematics,
      xController,
      yController,
      thetaController,
      this::setModuleStates,
      this);
    return swerveControllerCommand.andThen(() -> stop());
  }

  public void monitorClick() {
    if(Math.hypot(m_gyro.getRawAccelX(), 
      m_gyro.getRawAccelY()) 
        > 1.8) {
      System.out.println("Click");
    }
  }

  @Override
  public void periodic() {
    updateOdometry();
    monitorClick();
    SmartDashboard.putString("Position", m_odometry.getPoseMeters().toString());
  }
}
