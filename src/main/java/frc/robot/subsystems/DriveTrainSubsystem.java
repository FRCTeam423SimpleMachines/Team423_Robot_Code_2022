package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;


public class DriveTrainSubsystem extends SubsystemBase{

    private final CANSparkMax leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    private final CANSparkMax leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    private final CANSparkMax leftMotor3 = new CANSparkMax(DriveConstants.kLeftMotor3Port, MotorType.kBrushless);
    private final CANSparkMax rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    private final CANSparkMax rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);
    private final CANSparkMax rightMotor3 = new CANSparkMax(DriveConstants.kRightMotor3Port, MotorType.kBrushless);

    // The motors on the left side of the drive.
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
        leftMotor1, leftMotor2, leftMotor3);

    // The motors on the right side of the drive.
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
        rightMotor1, rightMotor2, rightMotor3);

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    private RelativeEncoder m_encoderL = leftMotor2.getEncoder();
    private RelativeEncoder m_encoderR = rightMotor2.getEncoder();

    /** Creates a new DriveSubsystem. */
    public DriveTrainSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightMotors.setInverted(true);

        // Sets the distance per pulse for the encoders
        //m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        //m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    }
    

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }
    
    /**
   * Returns the left encoder distance.
   *
   * @return the left encoder distance
   */
  public double getLeftEncoderDistance() {
    return m_encoderL.getPosition();
  }

  /**
   * Returns the right encoder distance.
   *
   * @return the right encoder distance
   */
  public double getRightEncoderDistance() {
    return m_encoderR.getPosition();
  }

  /** Resets the drive encoders. */
  public void resetEncoders() {
    m_encoderL.setPosition(0.0);
    m_encoderR.setPosition(0.0);
  }

     /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }


}
