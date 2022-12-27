// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.FollowTrajectoryWithEvents;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithValues;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.OIConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Theta is used in the DriveWithValues command to prevent the robot from overriding the SlewRateLimiters and precision controls
  public static double theta;

  private final XboxController m_driverController = new XboxController(kDriverControllerID);
    final JoystickButton faceForwardsButton = new JoystickButton(m_driverController, kYButtonID);
    final JoystickButton faceLeftButton = new JoystickButton(m_driverController, kXButtonID);
    final JoystickButton faceBackwardsButton = new JoystickButton(m_driverController, kAButtonID);
    final JoystickButton faceRightButton = new JoystickButton(m_driverController, kBButtonID);
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final DriveWithValues m_driveWithValues = new DriveWithValues(
    m_drivetrain,
    () -> m_driverController.getLeftX(),
    () -> m_driverController.getLeftY(),
    () -> theta,
    () -> m_driverController.getRightTriggerAxis(),
    () -> m_driverController.getLeftTriggerAxis() > 0.5
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrain.setDefaultCommand(
      new DriveWithJoysticks(
        m_drivetrain, 
        () -> m_driverController.getRightX(),
        () -> m_driverController.getRightTriggerAxis(),
        m_driveWithValues)
    );
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    faceForwardsButton.whileTrue(new TurnToAngle(m_drivetrain, 
      0.0,
      m_driveWithValues
      ));
    
    faceLeftButton.whileTrue(new TurnToAngle(m_drivetrain, 
      -90.0,
      m_driveWithValues
      ));
    
    faceRightButton.whileTrue(new TurnToAngle(m_drivetrain, 
      90.0,
      m_driveWithValues
      ));
    
    faceBackwardsButton.whileTrue(new TurnToAngle(m_drivetrain, 
      180.0,
      m_driveWithValues
      ));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new FollowTrajectoryWithEvents(m_drivetrain, "Test2");
  }
}
