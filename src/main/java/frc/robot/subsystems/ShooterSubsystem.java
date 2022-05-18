package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX shooterMotor = new WPI_TalonFX(ShooterConstants.kShooterMotorPort);

    private double maxSpeed;

    private boolean shooterSfty = false;

    ShuffleboardTab shooterTab = Shuffleboard.getTab("ShooterTab");
    NetworkTableEntry shooterSafety = shooterTab.add("Shooter Safety", shooterSfty).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    shooterTab.add("ShooterTab", this);

    shooterTab.addNumber("Shooter/Shooter Max Speed", () -> maxSpeed);
  }

  public void SetShooterMaxSpeed(double maxSpd) {
    shooterMotor.configPeakOutputForward(maxSpd);
    maxSpeed = maxSpd;

  }

  public double GetShooterSpeedAtDistance(double distance){

    double theta = ShooterConstants.kShooterAngle;
    double x = distance;
    double y0 = ShooterConstants.kShooterHeight;
    double k = 1.0/ShooterConstants.kShooterEfficiency;

    return k*8*Math.pow(3,0.5)*Math.pow(-1/(Math.cos((Math.PI*theta)/180)*(36*Math.sin((Math.PI*theta)/180) - 104*Math.cos((Math.PI*theta)/180) + y0*Math.cos((Math.PI*theta)/180) + x*Math.sin((Math.PI*theta)/180))),0.5)*(x + 36);
  }

  public void RunShooterControlled(){

  }

  public void RunShooter() {
    if (!shooterSfty) {
      shooterMotor.set(maxSpeed);
    }
    else {
      shooterMotor.set(0);
    }
  }

  public void logToDashboard() {
    //SmartDashboard.putNumber("Shooter/Shooter Speed (RPM)", shooterMotor.getSelectedSensorVelocity()/ShooterConstants.kencoderCPR);
    //SmartDashboard.putNumber("Shooter/Shooter Max Speed", maxSpeed);
    
    
  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logToDashboard();
    shooterSfty = shooterSafety.getBoolean(false);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }
}