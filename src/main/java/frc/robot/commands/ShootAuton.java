package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAuton extends SequentialCommandGroup{
    
    public ShootAuton(DriveTrainSubsystem drive, ShooterSubsystem shooter, BallElevator elevator) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
          // Start shooter at full speed
          // Drive 7.4 ft backwards
          // Shoot ball
          new SequentialCommandGroup(new ResetEncodersCommand(drive),
          
          new RunShooterHigh(shooter),
          new DriveDistanceProfiled(-7.4*12, drive)),
          new RunShooterInput(elevator)
          
        );
      }

}
