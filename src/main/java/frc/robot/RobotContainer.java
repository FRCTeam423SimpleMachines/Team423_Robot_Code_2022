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
import frc.robot.commands.DriveDistanceProfiled;
import frc.robot.commands.SimpleAuton;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem m_driveTrainSubsystem = new DriveTrainSubsystem();
  private final LiftSubsystem m_LiftSubsystem = new LiftSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();




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
      new RunCommand(() -> m_driveTrainSubsystem.arcadeDrive(deadbandJoystick(m_driverController.getY()), deadbandJoystick(m_driverController.getZ()) ), m_driveTrainSubsystem)
    );
    m_shooterSubsystem.setDefaultCommand(
      new RunCommand(() -> m_shooterSubsystem.RunShooter(), m_shooterSubsystem)
    );
    m_turretSubsystem.setDefaultCommand(
      new RunCommand(() -> m_turretSubsystem.turretAim(deadbandJoystick(m_driverController2.getZ())), m_turretSubsystem)
    );
    m_LiftSubsystem.setDefaultCommand(new RunCommand(() 
    -> m_LiftSubsystem.DontRun(), m_LiftSubsystem));

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Simple Auto", new SimpleAuton(m_driveTrainSubsystem));
    //m_chooser.addOption("Complex Auto", m_complexAuto);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    
    // Add Info Graphs on the dashboard
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("Arcade Drive", m_driveTrainSubsystem);

    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.add("Shooter", m_shooterSubsystem);

    // Put both encoders in a list layout
    ShuffleboardLayout encoders = driveBaseTab.getLayout("List Layout", "Encoders").withPosition(0, 0).withSize(2, 2);
    encoders.add("Left Encoder", m_driveTrainSubsystem.getLeftEncoderDistance());
    encoders.add("Right Encoder", m_driveTrainSubsystem.getRightEncoderDistance());


  }

  public double deadbandJoystick(double input) {
    if(Math.abs(input) < Constants.OIConstants.kDeadband) return 0;
    else return input;
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
      new JoystickButton(m_driverController, 12).whenPressed(new DriveDistanceProfiled(96.0, m_driveTrainSubsystem));
      new JoystickButton(m_driverController, 3).whenPressed(new TurnToAngleProfiled(90.0, m_driveTrainSubsystem));
      new JoystickButton(m_driverController, 4).whenPressed(new TurnToAngleProfiled(-90.0, m_driveTrainSubsystem));
      new JoystickButton(m_driverController, 8).whenHeld(new InstantCommand(()
      -> m_LiftSubsystem.RunUp(), m_LiftSubsystem));
      new JoystickButton(m_driverController, 10).whenHeld(new InstantCommand(()
      -> m_LiftSubsystem.RunDown(), m_LiftSubsystem));
      new JoystickButton(m_driverController, 11).whenPressed(new InstantCommand(()
        -> m_shooterSubsystem.SetShooterMaxSpeed(1.0), m_shooterSubsystem));
      new JoystickButton(m_driverController, 9).whenPressed(new InstantCommand(()
        -> m_shooterSubsystem.SetShooterMaxSpeed(0.0), m_shooterSubsystem));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    
    //CommandScheduler.getInstance().clearButtons();

    m_driveTrainSubsystem.resetEncoders();
    m_driveTrainSubsystem.resetGyro();

    return m_chooser.getSelected();

  }
}
