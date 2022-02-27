package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);

    private RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

    private float maxSpeed;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  public void SetShooterMaxSpeed(float maxSpd) {
    shooterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    shooterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) 5700.0*maxSpd);
    maxSpeed = maxSpd;

  }

  public void RunShooter() {
    shooterMotor.set(maxSpeed);
  }

  public void logToDashboard() {
    SmartDashboard.putNumber("Shooter/Shooter Speed", shooterEncoder.getVelocity());
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