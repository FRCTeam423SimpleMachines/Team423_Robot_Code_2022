package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretSlow extends CommandBase{
    
    TurretSubsystem m_turret;
    DoubleSupplier m_turnValue;

    public TurretSlow(TurretSubsystem turret, DoubleSupplier turnValue) {
        m_turret = turret;
        m_turnValue = turnValue;
        addRequirements(m_turret);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_turret.turretAim(m_turnValue.getAsDouble()*0.3);
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

