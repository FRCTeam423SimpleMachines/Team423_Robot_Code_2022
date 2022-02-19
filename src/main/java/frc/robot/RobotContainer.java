// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.*;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /// 
  // The autonomous routines
  ///

  // A simple auto routine that drives forward a specified distance, and then stops.
  //private final Command m_simpleAuto = new DriveDistance(AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed, m_robotDrive);



  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_driverController2 = new Joystick(OIConstants.kDriverControllerPort2);
  
  



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();

    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    m_driveTrainSubsystem.setDefaultCommand(
      new RunCommand(() -> m_driveTrainSubsystem.arcadeDrive(m_driverController.getY(), m_driverController.getZ() ), m_driveTrainSubsystem)
    );

    // Add commands to the autonomous command chooser
    //m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    //m_chooser.addOption("Complex Auto", m_complexAuto);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    
    // Add Info Graphs on the dashboard
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("Arcade Drive", m_driveTrainSubsystem);

    // Put both encoders in a list layout
    ShuffleboardLayout encoders = driveBaseTab.getLayout("List Layout", "Encoders").withPosition(0, 0).withSize(2, 2);
    encoders.add("Left Encoder", m_driveTrainSubsystem.getLeftEncoderDistance());
    encoders.add("Right Encoder", m_driveTrainSubsystem.getRightEncoderDistance());


  }

  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      new JoystickButton(m_driverController, 5).whenPressed(new InstantCommand(()
        -> m_driveTrainSubsystem.resetEncoders(), m_driveTrainSubsystem));
      new JoystickButton(m_driverController, 6).whenPressed(new InstantCommand(()
        -> m_driveTrainSubsystem.resetGyro(), m_driveTrainSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
