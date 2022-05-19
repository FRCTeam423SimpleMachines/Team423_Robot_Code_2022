// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import frc.robot.commands.DoNothingAuton;
import frc.robot.commands.DriveDistanceProfiled;
import frc.robot.commands.DumpAuton;
import frc.robot.commands.RunElevator;
import frc.robot.commands.RunIntakeDown;
import frc.robot.commands.RunIntakeUp;
import frc.robot.commands.RunLiftDown;
import frc.robot.commands.RunLiftUp;
import frc.robot.commands.RunShooterInput;
import frc.robot.commands.ShootAuton;
import frc.robot.commands.ShootBall;
import frc.robot.commands.SimpleAuton;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.TurretLock;
import frc.robot.commands.TurretSlow;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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
  private final LiftSubsystem m_liftSubsystem = new LiftSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final BallElevator m_ballElevator = new BallElevator();

  // Safeties for the robot's subsysytems

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

    CameraServer.startAutomaticCapture();

    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    
    m_driveTrainSubsystem.setDefaultCommand(
    new RunCommand(() -> m_driveTrainSubsystem.arcadeDrive(0.9*squareInput(deadbandJoystick(m_driverController.getY())), -0.9*m_driverController.getZ() ), m_driveTrainSubsystem)
    );
    
    m_shooterSubsystem.setDefaultCommand(
    new RunCommand(() -> m_shooterSubsystem.RunShooter(), m_shooterSubsystem)
    );
    
    m_turretSubsystem.setDefaultCommand(
    new RunCommand(() -> m_turretSubsystem.turretAim(deadbandJoystick(m_driverController2.getZ())), m_turretSubsystem));
    
    m_liftSubsystem.setDefaultCommand(new RunCommand(() 
    -> m_liftSubsystem.DontRun(), m_liftSubsystem));

    m_ballElevator.setDefaultCommand(new RunCommand(() 
    -> m_ballElevator.stopShooterInput() , m_ballElevator));

    m_intakeSubsystem.setDefaultCommand(new RunCommand(()
    -> m_intakeSubsystem.intakeArmStop(), m_intakeSubsystem));

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Simple Auto", new SimpleAuton(m_driveTrainSubsystem));
    m_chooser.addOption("Do Nothing", new DoNothingAuton(m_driveTrainSubsystem));
    m_chooser.addOption("Drive back and Shoot Ball", new ShootAuton(m_driveTrainSubsystem, m_shooterSubsystem, m_ballElevator));
    m_chooser.addOption("Dump Ball and drive back", new DumpAuton(m_driveTrainSubsystem, m_shooterSubsystem, m_ballElevator));
    //m_chooser.addOption("Complex Auto", m_complexAuto);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    
    // Add Info Graphs on the dashboard
    //ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    //driveBaseTab.add("Arcade Drive", m_driveTrainSubsystem);

    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.add("Shooter", m_shooterSubsystem);

    ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
    elevatorTab.add("Elevator", m_ballElevator);

    ShuffleboardTab liftTab = Shuffleboard.getTab("Lift");
    liftTab.add("Lift", m_liftSubsystem);

    // Put both encoders in a list layout
    //ShuffleboardLayout encoders = driveBaseTab.getLayout("List Layout", "Encoders").withPosition(0, 0).withSize(2, 2);
    //encoders.add("Left Encoder", m_driveTrainSubsystem.getLeftEncoderDistance());
    //encoders.add("Right Encoder", m_driveTrainSubsystem.getRightEncoderDistance());


  }

  public double deadbandJoystick(double input) {
    if(Math.abs(input) < Constants.OIConstants.kDeadband) return 0;
    else return input;
  }

  public double squareInput(double input){
      return input/Math.abs(input)*Math.pow(input, 2);
  }

  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
   /** 
   * Main Driver Controlls:
   * 5: Lift Up
   * 3: Lift Down
   * 6: Itake Up + Stop
   * 4: Intake Down + Run
   * 7: Reset Encoders
   * 8: Reset Gyro
   * Sub Driver Controlls:
   * 1: Shoot Ball
   * 2: Slow Turret
   * 5: Run Shooter (High)
   * 4; Run Shooter (Low)
   * 3: Stop Shooter
   * 8: Toggle Elavator
   * 10: Run Elavator Foward (Only When Elavator isn't running itself)
   * 12: Run Elavator Backward (Only When Elavator isn't running itself)
  */

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 7).whenPressed(new InstantCommand(()
      -> m_driveTrainSubsystem.resetEncoders(), m_driveTrainSubsystem));

    new JoystickButton(m_driverController, 8).whenPressed(new InstantCommand(()
      -> m_driveTrainSubsystem.resetGyro(), m_driveTrainSubsystem));

    new JoystickButton(m_driverController, 5).whenHeld(new RunLiftUp(m_liftSubsystem));

    new JoystickButton(m_driverController, 3).whenHeld(new RunLiftDown(m_liftSubsystem));

    new JoystickButton(m_driverController2, 1).whenHeld(new ShootBall(m_ballElevator));

    //new JoystickButton(m_driverController2, 8).whenPressed(new InstantCommand(()
    //-> m_ballElevator.EllavotorToggle(), m_ballElevator));

    //new JoystickButton(m_driverController2, 5).whenPressed(new InstantCommand(()
      //-> m_shooterSubsystem.SetShooterMaxSpeed(0.8), m_shooterSubsystem));

    new JoystickButton(m_driverController2, 3).whenPressed(new InstantCommand(()
      -> m_shooterSubsystem.SetShooterMaxSpeed(0.0), m_shooterSubsystem));

    new JoystickButton(m_driverController2, 4).whenPressed(new InstantCommand(()
      -> m_shooterSubsystem.SetShooterMaxSpeed(0.3), m_shooterSubsystem));

    new JoystickButton(m_driverController, 6).whenHeld(new RunIntakeUp(m_intakeSubsystem));

    new JoystickButton(m_driverController, 4).whenHeld(new RunIntakeDown(m_intakeSubsystem));

      new JoystickButton(m_driverController, 2).whenPressed(new InstantCommand(()
      -> m_intakeSubsystem.intakeArmStop(), m_intakeSubsystem));

    new JoystickButton(m_driverController2, 10).whenHeld(new InstantCommand(()
    -> m_ballElevator.runElevatorReverse(), m_ballElevator));

    new JoystickButton(m_driverController2, 12).whenHeld(new InstantCommand(()
      -> m_ballElevator.runElevatorForward(), m_ballElevator));
  
    new JoystickButton(m_driverController2, 1).whileHeld(new RunShooterInput(m_ballElevator));

    new JoystickButton(m_driverController, 12).whenPressed(new InstantCommand(()
    -> m_intakeSubsystem.intakeStop(), m_intakeSubsystem));

    new JoystickButton(m_driverController, 10).whenPressed(new InstantCommand(()
    -> m_intakeSubsystem.intakeRun(), m_intakeSubsystem));

    new JoystickButton(m_driverController2, 6).whenHeld(new TurretLock(m_turretSubsystem));

    new JoystickButton(m_driverController2, 2).whenHeld(new TurretSlow(m_turretSubsystem, ()-> deadbandJoystick(m_driverController2.getZ())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    
    //CommandScheduler.getInstance().clearButtons();

    //m_driveTrainSubsystem.resetEncoders();
    //m_driveTrainSubsystem.resetGyro();

    return m_chooser.getSelected();

  }
}
