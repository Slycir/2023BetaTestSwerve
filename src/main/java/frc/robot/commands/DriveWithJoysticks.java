// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveWithJoysticks extends CommandBase {

  Drivetrain m_drivetrain;

  DoubleSupplier m_theta;
  DoubleSupplier m_precision;

  BooleanSupplier m_robotRelative;

  Command m_driveWithValues;

  boolean isRobotRelative;

  double m_xSpeed;
  double m_ySpeed;
  double m_thetaSpeed;
  double m_precisionFactor;

  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(1 / Constants.DriveConstants.kAccelerationSeconds);
  /** Creates a new Drive. */
  public DriveWithJoysticks(Drivetrain drivetrain, DoubleSupplier theta, DoubleSupplier precision, Command driveWithValues) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;

    m_theta = theta;
    m_precision = precision;

    m_driveWithValues = driveWithValues;

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_precisionFactor = Math.pow(0.4, m_precision.getAsDouble());

    m_thetaSpeed =
      m_thetaLimiter.calculate(MathUtil.applyDeadband(m_theta.getAsDouble(), Constants.DriveConstants.kDriveDeadband))
      * Constants.DriveConstants.kMaxAngularSpeedRadiansPerSecond * Constants.DriveConstants.kSpeedFactor * m_precisionFactor;

    RobotContainer.theta = m_thetaSpeed;

    m_driveWithValues.schedule();
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
