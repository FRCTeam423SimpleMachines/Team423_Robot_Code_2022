package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final WPI_TalonFX shooterMotor = new WPI_TalonFX(ShooterConstants.kShooterMotorPort);

    private double maxSpeed;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public void SetShooterMaxSpeed(double maxSpd) {
    shooterMotor.configPeakOutputForward(maxSpd);
    maxSpeed = maxSpd;

  }

  public void RunShooter() {
    shooterMotor.set(maxSpeed);
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("Shooter/Shooter Speed", shooterMotor.getSelectedSensorVelocity());
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