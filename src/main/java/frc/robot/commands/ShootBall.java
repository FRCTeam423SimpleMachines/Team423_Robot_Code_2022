package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallElevator;

public class ShootBall extends CommandBase{
    
    BallElevator m_elevator;

    public ShootBall(BallElevator elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elevator.runElevatorForward();
        m_elevator.runShooterInputForward();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

}
