// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DriveConstants.*;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TurnToAngle extends CommandBase {

  Drivetrain m_drivetrain;
  Double targetAngle;
  private PIDController turnController = new PIDController(kTurnP, kTurnI, kTurnD);

  // private final SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(2 / kAccelerationSeconds);
  /** Creates a new ZeroHeading. */
  public TurnToAngle(Drivetrain drivetrain, Double angle, Command driveWithValues) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    targetAngle = angle;
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

    var rot = -turnController.calculate(m_drivetrain.getOdoYaw());
    rot = MathUtil.clamp(rot, -kMaxSpeedMetersPerSecond * kSpeedFactor, kMaxSpeedMetersPerSecond * kSpeedFactor);

    RobotContainer.theta = rot;


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
