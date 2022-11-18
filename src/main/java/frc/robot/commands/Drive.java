// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Drive extends CommandBase {

  Drivetrain m_drivetrain;

  DoubleSupplier m_x;
  DoubleSupplier m_y;
  DoubleSupplier m_theta;

  double m_xSpeed;
  double m_ySpeed;
  double m_thetaSpeed;

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / Constants.DriveConstants.kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / Constants.DriveConstants.kAccelerationSeconds);
  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(1 / Constants.DriveConstants.kAccelerationSeconds);
  /** Creates a new Drive. */
  public Drive(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;

    m_x = x;
    m_y = y;
    m_theta = theta;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_xSpeed =
      -m_xLimiter.calculate(MathUtil.applyDeadband(m_y.getAsDouble(), Constants.DriveConstants.kDriveDeadband))
      * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.DriveConstants.kSpeedFactor;
    
    m_ySpeed =
      -m_yLimiter.calculate(MathUtil.applyDeadband(m_x.getAsDouble(), Constants.DriveConstants.kDriveDeadband))
      * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.DriveConstants.kSpeedFactor;

    m_thetaSpeed =
      -m_thetaLimiter.calculate(MathUtil.applyDeadband(m_theta.getAsDouble(), Constants.DriveConstants.kDriveDeadband))
      * Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecond * Constants.DriveConstants.kSpeedFactor;

    m_drivetrain.drive(m_xSpeed, m_ySpeed, m_thetaSpeed);
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
