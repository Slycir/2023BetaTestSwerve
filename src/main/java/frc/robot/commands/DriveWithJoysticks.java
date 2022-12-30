// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.DriveConstants.*;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveWithJoysticks extends CommandBase {

  Drivetrain m_drivetrain;

  DoubleSupplier m_x;
  DoubleSupplier m_y;
  DoubleSupplier m_theta;
  DoubleSupplier m_precision;
  
  BooleanSupplier m_robotRelative;
  
  boolean m_PIDcontrol;
  boolean isRobotRelative;
  
  JoystickButton m_forwards;
  JoystickButton m_left;
  JoystickButton m_right;
  JoystickButton m_backwards;
  
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);
  
  double m_toAngle;
  double m_xSpeed;
  double m_ySpeed;
  double m_thetaSpeed;
  double m_precisionFactor;

  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(1 / kAccelerationSeconds);
  /** Creates a new Drive. */
  public DriveWithJoysticks(
      Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta, DoubleSupplier precision, BooleanSupplier robotRelative, 
      JoystickButton forwards, JoystickButton left, JoystickButton right, JoystickButton backwards
      ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;

    m_x = x;
    m_y = y;
    m_theta = theta;
    m_robotRelative = robotRelative;
    m_precision = precision;

    m_forwards = forwards;
    m_left = left;
    m_right = right;
    m_backwards = backwards;
    
    isRobotRelative = false;

    turnController.enableContinuousInput(-180, 180);

    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Facing button setup
    if(m_forwards.getAsBoolean()) {
      m_toAngle = 0.0;
      m_PIDcontrol = true;
    } else if(m_left.getAsBoolean()) {
      m_toAngle = -90.0;
      m_PIDcontrol = true;
    } else if(m_right.getAsBoolean()) {
      m_toAngle = 90.0;
      m_PIDcontrol = true;
    } else if(m_backwards.getAsBoolean()) {
      m_toAngle = 180.0;
      m_PIDcontrol = true;
    } else {
      m_toAngle = 0.0;
      m_PIDcontrol = false;
    }

    m_precisionFactor = Math.pow(0.4, m_precision.getAsDouble());
    m_xSpeed =
      -m_xLimiter.calculate(MathUtil.applyDeadband(m_y.getAsDouble(), kDriveDeadband))
      * kMaxSpeedMetersPerSecond * kSpeedFactor * m_precisionFactor;
    
    m_ySpeed =
      -m_yLimiter.calculate(MathUtil.applyDeadband(m_x.getAsDouble(), kDriveDeadband))
      * kMaxSpeedMetersPerSecond * kSpeedFactor * m_precisionFactor;

    if(m_PIDcontrol) {
      // PID control
      turnController.setSetpoint(m_toAngle);
      m_thetaSpeed = -turnController.calculate(m_drivetrain.getOdoYaw());
      m_thetaSpeed = MathUtil.clamp(m_thetaSpeed, -kMaxSpeedMetersPerSecond * kSpeedFactor, kMaxSpeedMetersPerSecond * kSpeedFactor);
    } else {
      // Joystick control
      m_thetaSpeed =
        -m_thetaLimiter.calculate(MathUtil.applyDeadband(m_theta.getAsDouble(), kDriveDeadband))
        * kMaxAngularSpeedRadiansPerSecond * kSpeedFactor * m_precisionFactor;
    }

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
