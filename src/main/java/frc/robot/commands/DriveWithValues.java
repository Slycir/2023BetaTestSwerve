// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveWithValues extends CommandBase {

  Drivetrain m_drivetrain;

  DoubleSupplier m_x;
  DoubleSupplier m_y;
  DoubleSupplier m_theta;
  DoubleSupplier m_precision;

  BooleanSupplier m_robotRelative;

  boolean isRobotRelative;

  double m_xSpeed;
  double m_ySpeed;
  double m_thetaSpeed;
  double m_precisionFactor;

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / Constants.DriveConstants.kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / Constants.DriveConstants.kAccelerationSeconds);
  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(1 / Constants.DriveConstants.kAccelerationSeconds);
  /** Creates a new Drive. */
  public DriveWithValues(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, DoubleSupplier precision, BooleanSupplier robotRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;

    m_x = x;
    m_y = y;
    m_theta = theta;
    m_robotRelative = robotRelative;
    m_precision = precision;

    isRobotRelative = false;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_precisionFactor = Math.pow(0.4, m_precision.getAsDouble());
    m_xSpeed =
      -m_xLimiter.calculate(MathUtil.applyDeadband(m_y.getAsDouble(), Constants.DriveConstants.kDriveDeadband))
      * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.DriveConstants.kSpeedFactor * m_precisionFactor;
    
    m_ySpeed =
      -m_yLimiter.calculate(MathUtil.applyDeadband(m_x.getAsDouble(), Constants.DriveConstants.kDriveDeadband))
      * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.DriveConstants.kSpeedFactor * m_precisionFactor;

    m_thetaSpeed =
      -m_theta.getAsDouble();

    if(m_robotRelative.getAsBoolean() != isRobotRelative) {
      if(m_xSpeed == 0.0 && m_ySpeed == 0.0) {
        isRobotRelative = m_robotRelative.getAsBoolean();
      }
    }

    SmartDashboard.putBoolean("Robot Relative", isRobotRelative);

    m_drivetrain.drive(m_xSpeed, m_ySpeed, m_thetaSpeed, !isRobotRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
