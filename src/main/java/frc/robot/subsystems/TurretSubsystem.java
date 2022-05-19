package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  
  private final CANSparkMax turretMotor = new CANSparkMax(TurretConstants.kTurretMotorPort, MotorType.kBrushless);
  private RelativeEncoder turretEncoder = turretMotor.getEncoder();
  DigitalInput magLimitSwitch = new DigitalInput(TurretConstants.kTurrentSensorPort);

  private boolean turretSfty = true;

  ShuffleboardTab turretTab = Shuffleboard.getTab("TurretTab");
  NetworkTableEntry turretSafety = Shuffleboard.getTab("Safeties").add("Turret Safety", turretSfty).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

  /** Creates a new ExampleSubsystem. */
  public TurretSubsystem() {
    

    turretTab.addNumber("Turret/Turret Speed", () -> turretEncoder.getVelocity());
    turretTab.addNumber("Turret/Turret Position", () -> turretEncoder.getPosition());
    turretTab.addNumber("Turret/Turret Angle", () -> getAngleFromEncoder(turretEncoder));
  }

  public void turretAim(double turn) {
    if (turretSfty) {
    if (turretEncoder.getPosition()<141.6 && turretEncoder.getPosition()>-129) {
      turretMotor.set(turn);
    }
    else if (turretEncoder.getPosition()>=141.6) {
      if (turn < 0) {
        turretMotor.set(turn);
      } else {
        turretMotor.set(0.0);
      }
    }
    else if (turretEncoder.getPosition()<=-129) {
      if (turn > 0) {
      turretMotor.set(turn);
      } else {
        turretMotor.set(0.0);
      }
    }
    //turretMotor.set(turn);
    }
  }

  
 public void stopTurret() {
   turretMotor.set(0.0);
 }


  public double getAngleFromEncoder(RelativeEncoder encoder) {
    return encoder.getPosition()*TurretConstants.CLICKS_PER_DEGREE;
  }

  public void logToDashboard() {
    //SmartDashboard.putNumber("Turret/Turret Speed", turretEncoder.getVelocity());
    //SmartDashboard.putNumber("Turret/Turret Position", turretEncoder.getPosition());
    //SmartDashboard.putNumber("Turret/Turret Angle", getAngleFromEncoder(turretEncoder));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logToDashboard();
    turretSfty = turretSafety.getBoolean(true);

    if (!magLimitSwitch.get()) {
      turretEncoder.setPosition(0.0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}