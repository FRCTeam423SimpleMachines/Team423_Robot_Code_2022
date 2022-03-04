package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BallElevatorConstants.BallStates;
import frc.robot.subsystems.BallElevator;

public class RunElevator extends CommandBase{
    
    BallElevator m_elevator;

    public RunElevator(BallElevator elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_elevator.getBallState() == BallStates.TWO){
            m_elevator.stopElevator();
            m_elevator.stopShooterInput();
        } else if (m_elevator.getBallState() == BallStates.HIGH){
            m_elevator.runElevatorForward();
            m_elevator.stopShooterInput();
        } else if (m_elevator.getBallState() == BallStates.LOW){
            m_elevator.runElevatorForward();
            m_elevator.runShooterInputForward();
        } else {
            m_elevator.runElevatorForward();
            m_elevator.runShooterInputForward();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
