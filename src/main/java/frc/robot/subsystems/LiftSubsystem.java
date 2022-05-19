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
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.LiftConstants.LiftStates;

public class LiftSubsystem extends SubsystemBase {

  private final CANSparkMax liftMotor = new CANSparkMax(LiftConstants.kLiftMotorPort, MotorType.kBrushless);

  private RelativeEncoder liftEncoder = liftMotor.getEncoder();

  DigitalInput bottomLimitSwitch = new DigitalInput(LiftConstants.kLiftBottomLimitSwitchPort);
  DigitalInput topLimitSwitch = new DigitalInput(LiftConstants.kLiftTopLimitSwitchPort);

  private LiftConstants.LiftStates state = LiftConstants.LiftStates.BOTTOM;

  private boolean liftSfty = true;

  ShuffleboardTab liftTab = Shuffleboard.getTab("LiftTab");
  NetworkTableEntry liftSafety = Shuffleboard.getTab("Safeties").add("Lift Safety", liftSfty).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

  /** Creates a new ExampleSubsystem. */
  public LiftSubsystem() {
    liftTab.add("LiftTab", this);

    liftTab.addNumber("Lift/Lift Position", () -> liftEncoder.getPosition());
    liftTab.addNumber("Lift/Lift Speed", () -> liftEncoder.getVelocity());
    liftTab.addString("Lift/Lift State", () -> state.toString());
    liftTab.addBoolean("Lift/Top Limit", () -> topLimitSwitch.get());
    liftTab.addBoolean("Lift/Bottom Limit", () -> bottomLimitSwitch.get());
  }

  public void DontRun() {
      liftMotor.set(0.0);
  }

  public void RunUp() {
    if (liftSfty) {
      if (state != LiftStates.TOP) {
        liftMotor.set(0.8);
      }
    }
  }
  
  public void RunDown() {
    if (liftSfty) {
      if (state != LiftStates.BOTTOM) {
        liftMotor.set(-0.8);
      }
    }  
  }

  public void logToDashboard() {
      
  }

  public void updateState(){
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateState();
    logToDashboard();
    liftSfty = liftSafety.getBoolean(true);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}
