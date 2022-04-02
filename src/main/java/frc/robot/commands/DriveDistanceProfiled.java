/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.NotMath;

/**
 * A command that will drive the robot to the specified angle distance a motion profile.
 */
public class DriveDistanceProfiled extends ProfiledPIDCommand {
  /**
   * Creates a new TurnToAngleProfiled.
   */
  
  /**
   * Drives to robot to the specified angle using a motion profile.
   *
   * @param targetDistanceInches The angle to turn to
   * @param drive              The drive subsystem to use
   */

  DriveTrainSubsystem m_drive;
  private int count = 0;
  public DriveDistanceProfiled(double targetDistanceInches, DriveTrainSubsystem drive) {

    super(
        new ProfiledPIDController(
            Constants.DriveConstants.PROFILED_DRIVE_P, 
            Constants.DriveConstants.PROFILED_DRIVE_I,
            Constants.DriveConstants.PROFILED_DRIVE_D, 
            new TrapezoidProfile.Constraints(
                Constants.DriveConstants.MAX_DRIVE_RATE_IN_PER_S,
                Constants.DriveConstants.MAX_DRIVE_ACCELERATION_IN_PER_S_SQUARED
            )
        ),
        // Close loop on heading
        () -> drive.getAvrageEncoderDistance(),
        // Set reference to target
        targetDistanceInches/0.8,
        // Pipe output to drive robot
        (output, setpoint) -> drive.getDifferentialDrive().arcadeDrive( NotMath.minmax(-output, -1.0, 1.0),0),
        
        // Require the drive
        drive
    );

    m_drive = drive;
    // Set the controller to be continuous (because it is an angle controller)
    //getController().enableContinuousInput(-100000, 100000);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(Constants.DriveConstants.DRIVE_TOLERANCE_IN, Constants.DriveConstants.DRIVE_RATE_TOLERANCE_IN_PER_S);

    SmartDashboard.putNumber("DrivePID/P", Constants.DriveConstants.PROFILED_DRIVE_P);
    SmartDashboard.putNumber("DriveID/I", Constants.DriveConstants.PROFILED_DRIVE_I);
    SmartDashboard.putNumber("DriveID/D", Constants.DriveConstants.PROFILED_DRIVE_D);
  }

 @Override
 public void initialize() {




    // TODO Auto-generated method stub
    getController().reset(m_drive.getAvrageEncoderDistance());
    double p = SmartDashboard.getNumber("DrivePID/P", Constants.DriveConstants.PROFILED_DRIVE_P);
    double i = SmartDashboard.getNumber("DrivePID/I", Constants.DriveConstants.PROFILED_DRIVE_I);
    double d = SmartDashboard.getNumber("DrivePID/D", Constants.DriveConstants.PROFILED_DRIVE_D);
    getController().setPID(p, i, d);

    //m_drive.resetEncoders();
    count = 0;
    super.initialize();
   

 }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    m_drive.setSpeed(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    SmartDashboard.putNumber("DrivePID/PosError", getController().getPositionError());
    SmartDashboard.putNumber("DrivePID/VelError", getController().getVelocityError());
    SmartDashboard.putNumber("DrivePID/Goal", getController().getGoal().position);
    SmartDashboard.putNumber("DrivePID/Setpoint", getController().getSetpoint().position);
    SmartDashboard.putNumber("DrivePID/Supplied Distance", m_drive.getAvrageEncoderDistance());
    SmartDashboard.putNumber("DrivePID/Output", getController().calculate(m_drive.getAvrageEncoderDistance()));
    SmartDashboard.putNumber("DrivePID/Count", count);
    SmartDashboard.putBoolean("DrivePID/AtGoal", getController().atGoal());
    
    if (getController().atGoal()) {
      count++;
    } else {
      count = 0;
    }

    return count > 10;
  }
}