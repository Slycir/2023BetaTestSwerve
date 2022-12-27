// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {

  Drivetrain m_drivetrain;
  Double targetAngle;
  DoubleSupplier m_x, m_y;
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  double m_xSpeed;
  double m_ySpeed;

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  // private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(2 / kAccelerationSeconds);
  /** Creates a new ZeroHeading. */
  public TurnToAngle(Drivetrain drivetrain, Double angle, DoubleSupplier x, DoubleSupplier y) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    targetAngle = angle;
    m_x = x;
    m_y = y;
    addRequirements(m_drivetrain);
    turnController.setSetpoint(targetAngle);
    turnController.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_xSpeed =
      -m_xLimiter.calculate(MathUtil.applyDeadband(m_y.getAsDouble(), kDriveDeadband))
      * kMaxSpeedMetersPerSecond * kSpeedFactor;
  
    m_ySpeed =
      -m_yLimiter.calculate(MathUtil.applyDeadband(m_x.getAsDouble(), kDriveDeadband))
      * kMaxSpeedMetersPerSecond * kSpeedFactor;

    var rot = -turnController.calculate(m_drivetrain.getOdoYaw());
    rot = MathUtil.clamp(rot, -kMaxSpeedMetersPerSecond * kSpeedFactor, kMaxSpeedMetersPerSecond * kSpeedFactor);
    m_drivetrain.drive(m_xSpeed, m_ySpeed, rot);
    // m_drivetrain.drive(0, 0, -rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turnController.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
