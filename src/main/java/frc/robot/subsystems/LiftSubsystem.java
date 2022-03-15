package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.LiftConstants.LiftStates;

public class LiftSubsystem extends SubsystemBase {

  private final CANSparkMax liftMotor = new CANSparkMax(LiftConstants.kLiftMotorPort, MotorType.kBrushless);

  private RelativeEncoder liftEncoder = liftMotor.getEncoder();

  DigitalInput bottomLimitSwitch = new DigitalInput(LiftConstants.kLiftBottomLimitSwitchPort);
  DigitalInput topLimitSwitch = new DigitalInput(LiftConstants.kLiftTopLimitSwitchPort);

  private LiftConstants.LiftStates state = LiftConstants.LiftStates.BOTTOM;

  /** Creates a new ExampleSubsystem. */
  public LiftSubsystem() {}

  public void DontRun() {
      liftMotor.set(0.0);
  }

  public void RunUp() {
    if (state != LiftStates.TOP) {
      liftMotor.set(0.8);
    }
  }
  
  public void RunDown() {
    if (state != LiftStates.BOTTOM) {
      liftMotor.set(-0.8);
    }
  }

  public void logToDashboard() {
      SmartDashboard.putNumber("Lift/Lift Position", liftEncoder.getPosition());
      SmartDashboard.putNumber("Lift/Lift Speed", liftEncoder.getVelocity());
      SmartDashboard.putString("Lift/Lift State", state.toString());
      SmartDashboard.putBoolean("Lift/Top Limit", topLimitSwitch.get());
      SmartDashboard.putBoolean("Lift/Bottom Limit", bottomLimitSwitch.get());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if (!bottomLimitSwitch.get() || liftEncoder.getPosition() <= 0) {
      state = LiftStates.BOTTOM;
      liftEncoder.setPosition(0.0);
    }
    else if (!topLimitSwitch.get() || liftEncoder.getPosition() >= 140) {
      state = LiftStates.TOP;
      //liftEncoder.setPosition(140.0);
    }
    else {
      state = LiftStates.MIDDLE;
    }

    logToDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
