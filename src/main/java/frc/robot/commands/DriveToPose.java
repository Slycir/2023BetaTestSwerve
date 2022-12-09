// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DriveConstants;

public class DriveToPose extends CommandBase {

  Pose2d m_pose;
  Pose2d m_currentPose;

  Drivetrain m_drivetrain;

  PIDController m_xController = new PIDController(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);
  PIDController m_yController = new PIDController(DriveConstants.kDriveP, DriveConstants.kDriveI, DriveConstants.kDriveD);

  PIDController m_turnController = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);

  double m_xError;
  double m_yError;
  double m_turnError;
  /** Creates a new DriveToPose. */
  public DriveToPose(Drivetrain drivetrain, Pose2d pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_pose = pose;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_xController.setSetpoint(0.0);
    m_yController.setSetpoint(0.0);
    m_turnController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPose = m_drivetrain.getFieldPosition();
    m_xError = m_pose.getTranslation().getX() - m_currentPose.getTranslation().getX();
    m_yError = m_pose.getTranslation().getY() - m_currentPose.getTranslation().getY();
    m_turnError = m_pose.getRotation().getRadians() - m_currentPose.getRotation().getRadians();

    m_drivetrain.drive(clampToMaxVelocity(m_xController.calculate(m_xError)), clampToMaxVelocity(m_yController.calculate(m_yError)), clampToMaxAngularVelocity(m_turnController.calculate(m_turnError)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double clampToMaxVelocity(double velocity) {
    if (velocity > DriveConstants.kMaxSpeedMetersPerSecond) {
      return DriveConstants.kMaxSpeedMetersPerSecond;
    } else if (velocity < -DriveConstants.kMaxSpeedMetersPerSecond) {
      return -DriveConstants.kMaxSpeedMetersPerSecond;
    } else {
      return velocity;
    }
  }

  public double clampToMaxAngularVelocity(double velocity) {
    if (velocity > DriveConstants.kMaxAngularSpeedRadiansPerSecond) {
      return DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    } else if (velocity < -DriveConstants.kMaxAngularSpeedRadiansPerSecond) {
      return -DriveConstants.kMaxAngularSpeedRadiansPerSecond;
    } else {
      return velocity;
    }
  }
}
