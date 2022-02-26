package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  
  private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.kTurretMotorPort, MotorType.kBrushless);

  private RelativeEncoder TurretEncoder = turretMotor.getEncoder();

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem() {}

  public double getEncoderAngle(RelativeEncoder encoder) {
    double encoderValue = encoder.getPosition();
    return encoderValue*8.57142857143;
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("Turret/Turret Angle", getEncoderAngle(TurretEncoder));
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