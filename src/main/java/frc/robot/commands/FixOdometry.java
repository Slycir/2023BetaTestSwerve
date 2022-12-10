// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FixOdometry extends CommandBase {

  Drivetrain m_drivetrain;

  Pose2d TEST_TAG_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

  private static class TEST_OUTPUT {
    public static double Dx = 0;
    public static double Dy = 0;
    public static double Yaw = 0;
  }

  private double distance;
  private double absoluteAngle;
  private double relativeAngle;
  private double absoluteX;
  private double absoluteY;
  private double unsignedYaw;
  /** Creates a new FixOdometry. */
  public FixOdometry(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    unsignedYaw = unsigner(TEST_OUTPUT.Yaw);
    distance = Math.hypot(TEST_OUTPUT.Dx, TEST_OUTPUT.Dy);
    relativeAngle = unsigner(unsignedYaw - TEST_TAG_POSE.getRotation().getDegrees());
    absoluteAngle = ((relativeAngle) - Math.toDegrees(Math.atan(TEST_OUTPUT.Dy/TEST_OUTPUT.Dx))) % 360;
    absoluteX = TEST_TAG_POSE.getX() - (Math.sin(Math.toRadians(relativeAngle)) * distance); // TODO: Check this
    absoluteY = TEST_TAG_POSE.getY() - (Math.cos(Math.toRadians(relativeAngle)) * distance); // TODO: Check this
    m_drivetrain.setFieldPosition(new Pose2d(absoluteX, absoluteY, Rotation2d.fromDegrees(absoluteAngle)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public double unsigner(double angle) {
    if (angle < 0) {
      return angle + 360;
    } else {
      return angle;
    }
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
