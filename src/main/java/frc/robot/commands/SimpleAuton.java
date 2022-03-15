package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;

public class SimpleAuton extends SequentialCommandGroup{
    
    public SimpleAuton(DriveTrainSubsystem drive) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(
          // Drive 7.4 ft backwards
          new SequentialCommandGroup(new ResetEncodersCommand(drive),
          new DriveDistanceProfiled(-7.4*12, drive))
          
        );
      }

}
