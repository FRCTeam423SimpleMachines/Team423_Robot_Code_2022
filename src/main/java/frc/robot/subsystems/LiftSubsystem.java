package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class LiftSubsystem extends SubsystemBase {

  private final CANSparkMax liftMotor = new CANSparkMax(LiftConstants.kLiftMotorPort, MotorType.kBrushless);

  private RelativeEncoder liftEncoder = liftMotor.getEncoder();

  DigitalInput bottomLimitSwitch = new DigitalInput(LiftConstants.kBottomLimitSwitchPort);
  DigitalInput topLimitSwitch = new DigitalInput(LiftConstants.kTopLimitSwitchPort);

  boolean top;
  boolean bottom;

  /** Creates a new ExampleSubsystem. */
  public LiftSubsystem() {}

  public void DontRun() {
      liftMotor.set(0.0);
  }

  public void RunUp() {
    while (!top) {
      liftMotor.set(1.0);
    }
    liftMotor.set(0.0);
  }
  
  public void RunDown() {
    while (!bottom) {
      liftMotor.set(-1.0);
    }
    liftMotor.set(0.0);
  }

  public void logToDashboard() {
      SmartDashboard.putNumber("Lift/Lift Speed", liftEncoder.getVelocity());
      SmartDashboard.putBoolean("Lift/Lift At Top", top);
      SmartDashboard.putBoolean("Lift/Lift At Bottom", bottom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if (bottomLimitSwitch.get()&&topLimitSwitch.get()) {
      bottom = true;
      top = false;
    }
    else if (bottomLimitSwitch.get()) {
      bottom = false;
      top = true;
    }
    else {
      bottom = false;
      top = false;
    }

    logToDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
