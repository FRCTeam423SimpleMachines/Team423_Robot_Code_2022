package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
  private final CANSparkMax intakeArmMotor = new CANSparkMax(IntakeConstants.kIntakeArmMotorPort, MotorType.kBrushless);

  private RelativeEncoder m_IntakeEncoder = intakeMotor.getEncoder();
  private RelativeEncoder m_IntakeArmEncoder = intakeArmMotor.getEncoder();

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}