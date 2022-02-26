package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX turretMotor = new WPI_TalonFX(TurretConstants.kTurretMotorPort);
  DigitalInput magLimitSwitch = new DigitalInput(0);

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem() {
    turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("Turret/Turret Speed", turretMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Turret/Turret Position", turretMotor.getSelectedSensorPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logToDashboard();

    if (magLimitSwitch.get()) {
      turretMotor.setSelectedSensorPosition(0.0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}