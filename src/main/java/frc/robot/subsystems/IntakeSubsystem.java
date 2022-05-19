package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax intakeArmMotor = new CANSparkMax(IntakeConstants.kIntakeArmMotorPort, MotorType.kBrushless);

  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorPort);

  private IntakeConstants.IntakeStates state = IntakeConstants.IntakeStates.TOP;
  
  private RelativeEncoder intakeArmEncoder = intakeArmMotor.getEncoder();

  DigitalInput intakeTop = new DigitalInput(IntakeConstants.kIntakeTopPort);
  DigitalInput intakeBottom = new DigitalInput(IntakeConstants.kIntakeBottomPort);

  private boolean intakeSfty = true; 

  ShuffleboardTab intakeTab = Shuffleboard.getTab("IntakeTab");
  NetworkTableEntry intakeSafety = Shuffleboard.getTab("Safeties").add("Intake Safety", intakeSfty).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    intakeTab.addNumber("Intake/Intake Arm Position", () -> intakeArmEncoder.getPosition());
    intakeTab.addBoolean("Intake/Top", () -> intakeTop.get());
    intakeTab.addBoolean("Intake/Bottom", () -> intakeBottom.get());
    intakeTab.addString("Intake/State", () -> state.toString());
  }

  public void intakeUp() {
    if (intakeSfty) {
    //if (state != IntakeConstants.IntakeStates.TOP) {
      intakeArmMotor.set(1.0);
    //} else {
      //intakeArmMotor.set(0.0);
      //intakeMotor.set(0.0);
    //}
    }
  }

  public void intakeDown() {
    if (intakeSfty) {
    //if (state != IntakeConstants.IntakeStates.BOTTOM) {
      intakeArmMotor.set(-1.0);
    //} else {
      //intakeArmMotor.set(0.0);
      //intakeMotor.set(1.0); 
    //}
    }
  }

  public void intakeRun() {
    if (intakeSfty) {
    intakeMotor.set(1.0);
    }
  }

  public void intakeStop() {
    intakeMotor.set(0.0);
  }

  public void intakeArmStop() {
    intakeArmMotor.set(0.0);
  }

  public boolean getTopEncoder(){
    return intakeTop.get();
  }

  public boolean getBottomEncoder(){
    return intakeBottom.get();
  }

  public void logToDashboard() {
   

  }

  public void updateState() {
    if (!intakeBottom.get()) { 
      state = IntakeConstants.IntakeStates.BOTTOM;
      intakeArmEncoder.setPosition(0.0);
    }
    else if (!intakeTop.get() || intakeArmEncoder.getPosition()>=300) {
      state = IntakeConstants.IntakeStates.TOP;
    }
    else {
      state = IntakeConstants.IntakeStates.MIDDLE;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateState();
    logToDashboard();
    intakeSfty = intakeSafety.getBoolean(true);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}