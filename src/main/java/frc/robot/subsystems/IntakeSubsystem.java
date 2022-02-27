package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
  private final CANSparkMax intakeArmMotor = new CANSparkMax(IntakeConstants.kIntakeArmMotorPort, MotorType.kBrushless);

  private RelativeEncoder IntakeEncoder = intakeMotor.getEncoder();
  private RelativeEncoder IntakeArmEncoder = intakeArmMotor.getEncoder();

  DigitalInput intakeTop = new DigitalInput(3);
  DigitalInput intakeBottom = new DigitalInput(4);

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {}

  public void intakeUp() {
    while (!intakeTop.get()) {
      intakeArmMotor.set(0.8);
    }
    intakeArmMotor.set(0.0);
    intakeMotor.set(0.0);
  }

  public void intakeDown() {
    while (!intakeBottom.get()) {
      intakeArmMotor.set(-0.8);
    }
    intakeArmMotor.set(0.0);
    intakeMotor.set(1.0); 
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("Intake/Intake Speed", IntakeEncoder.getVelocity());
    SmartDashboard.putNumber("Intake/Intake Arm Position", IntakeArmEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logToDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}