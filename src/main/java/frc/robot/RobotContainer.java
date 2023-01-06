// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.FollowTrajectoryWithEvents;
import frc.robot.commands.ModuleCalibration;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.DriveWithJoysticks;
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

  private final XboxController m_driverController = new XboxController(kDriverControllerID);
    final JoystickButton faceForwardsButton = new JoystickButton(m_driverController, kYButtonID);
    final JoystickButton faceLeftButton = new JoystickButton(m_driverController, kXButtonID);
    final JoystickButton faceBackwardsButton = new JoystickButton(m_driverController, kAButtonID);
    final JoystickButton faceRightButton = new JoystickButton(m_driverController, kBButtonID);
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();

  public double m_toAngle = 0.0;
  public boolean m_PIDcontrol = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_drivetrain.setDefaultCommand(
      // new DriveWithJoysticks(
        // m_drivetrain,
        // () -> m_driverController.getLeftX(),
        // () -> m_driverController.getLeftY(),
        // () -> m_driverController.getRightX(),
        // () -> m_driverController.getRightTriggerAxis(),
        // () -> m_driverController.getLeftTriggerAxis() > 0.5,
        // faceForwardsButton,
        // faceLeftButton,
        // faceRightButton,
        // faceBackwardsButton
      // )
      new ModuleCalibration(m_drivetrain)
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
