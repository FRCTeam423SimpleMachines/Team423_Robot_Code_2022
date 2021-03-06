package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterLow extends CommandBase{
    
    ShooterSubsystem m_shooter;
    int count = 0;

    public RunShooterLow(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(shooter);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.RunShooter();
        m_shooter.SetShooterMaxSpeed(0.30);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        count++;
        if (count >= 10){
            return true;
        } else {
            return false;
        }
    }
}