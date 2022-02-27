package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  
  private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.kTurretMotorPort, MotorType.kBrushless);
  private RelativeEncoder turretEncoder = turretMotor.getEncoder();
  DigitalInput magLimitSwitch = new DigitalInput(0);

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem() {}

  public double getAngleFromEncoder(RelativeEncoder encoder) {
    return encoder.getPosition()/TurretConstants.CPA;
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("Turret/Turret Speed", turretEncoder.getVelocity());
    SmartDashboard.putNumber("Turret/Turret Position", turretEncoder.getPosition());
    SmartDashboard.putNumber("Turret/Turret Angle", getAngleFromEncoder(turretEncoder));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logToDashboard();

    if (magLimitSwitch.get()) {
      turretEncoder.setPosition(0.0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}