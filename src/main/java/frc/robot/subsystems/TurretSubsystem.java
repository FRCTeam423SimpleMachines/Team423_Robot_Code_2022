package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  
  private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.kTurretMotorPort, MotorType.kBrushless);

  private RelativeEncoder m_encoderT = turretMotor.getEncoder();

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}