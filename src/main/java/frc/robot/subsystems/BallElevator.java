package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallElevatorConstants;
import frc.robot.Constants.BallElevatorConstants.BallStates;

public class BallElevator extends SubsystemBase{
    
    private final CANSparkMax lowerElevatorMotor = new CANSparkMax(BallElevatorConstants.kLowerElevatorPort, MotorType.kBrushless);
    private final CANSparkMax upperElevatorMotor = new CANSparkMax(BallElevatorConstants.kUpperElevatorPort, MotorType.kBrushless);
    private final CANSparkMax shooterInputMotor = new CANSparkMax(BallElevatorConstants.kshooterInputPort, MotorType.kBrushless);
    
    
    private RelativeEncoder lowerElevatorEncoder = lowerElevatorMotor.getEncoder();
    private RelativeEncoder upperElevatorEncoder = upperElevatorMotor.getEncoder();
    private RelativeEncoder shooterInputEncoder = shooterInputMotor.getEncoder();

    //private Relay ballKickerRelay = new Relay(BallElevatorConstants.kballKickerPort);

    DigitalInput ballLow = new DigitalInput(BallElevatorConstants.kBallSwitch0);
    DigitalInput ballHigh = new DigitalInput(BallElevatorConstants.kBallSwitch1);

    private BallElevatorConstants.BallStates BallState = BallStates.ZERO;

  
    /** Creates a new ExampleSubsystem. */
    public BallElevator() {
        shooterInputMotor.setInverted(true);
        lowerElevatorMotor.setInverted(true);

        ShuffleboardTab elevatorTab = Shuffleboard.getTab("BallElevator");
        elevatorTab.add("BallElevator", this);

        Shuffleboard.getTab("BallElevator").addNumber("Elevator/Lower Elevator Speed", () -> lowerElevatorEncoder.getVelocity());
        Shuffleboard.getTab("BallElevator").addNumber("Elevator/Upper Elevator Speed", () -> upperElevatorEncoder.getVelocity());
        Shuffleboard.getTab("BallElevator").addNumber("Elevator/Shooter Input Speed", () -> shooterInputEncoder.getVelocity());
        elevatorTab.addString("Elevator/Ball Elevator State", () -> BallState.toString());
    
    }

    public void runElevatorForward(){
        lowerElevatorMotor.set(0.5);
        upperElevatorMotor.set(0.5);
    }

    public void runElevatorReverse(){
        lowerElevatorMotor.set(-0.5);
        upperElevatorMotor.set(-0.5);
    }

    public void stopElevator() {
        lowerElevatorMotor.set(0.0);
        upperElevatorMotor.set(0.0);
    }

    public void runShooterInputForward(){
        shooterInputMotor.set(0.5);
        //ballKickerRelay.set(Relay.Value.kForward);
    }

    public void runShooterInputReverse(){
        shooterInputMotor.set(-0.5);
        //ballKickerRelay.set(Relay.Value.kReverse);
    }

    public void stopShooterInput(){
        shooterInputMotor.set(0.0);
        //ballKickerRelay.set(Relay.Value.kOff);
    }


    public BallStates getBallState(){
        return BallState;
    }

    public void updateBallState(){
        if (ballHigh.get() && ballLow.get()){
            BallState = BallStates.TWO;
        } else if (ballHigh.get() && !ballLow.get()) {
            BallState = BallStates.HIGH;
        } else if (!ballHigh.get() && ballLow.get()) {
            BallState = BallStates.LOW;
        } else {
            BallState = BallStates.ZERO;
        }

    }
  
  
    public void logToDashboard() {
      
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      updateBallState();
      logToDashboard();
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }



}
