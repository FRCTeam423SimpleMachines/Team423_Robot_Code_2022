package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BallElevator;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DumpAuton extends SequentialCommandGroup{
    
    public DumpAuton(DriveTrainSubsystem drive, ShooterSubsystem shooter, BallElevator elevator) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
          // Start shooter at full speed
          // Drive 7.4 ft backwards
          // Shoot ball
          new SequentialCommandGroup(
            new ResetEncodersCommand(drive),
            new RunShooterLow(shooter),
            new RunShooterInput(elevator),
            new WaitCommand(5.0),
            new DriveDistanceProfiled(-7.4*12, drive))          
        );
      }

}
