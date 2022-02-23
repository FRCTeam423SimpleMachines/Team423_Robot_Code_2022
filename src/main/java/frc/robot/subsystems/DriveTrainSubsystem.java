package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;
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


    private AHRS mGyro;

    /** Creates a new DriveSubsystem. */
    public DriveTrainSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightMotors.setInverted(true);

        mGyro = new AHRS(SerialPort.Port.kMXP);

        leftMotor2.getEncoder().setVelocityConversionFactor(4*Math.PI/60);
        rightMotor2.getEncoder().setVelocityConversionFactor(4*Math.PI/60);

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
   * Attempts to follow the given drive states using offboard PID.
   *
   * @param left The left wheel state.
   * @param right The right wheel state.
   */
  public void setDriveStates(TrapezoidProfile.State left, TrapezoidProfile.State right) {
    /*leftMotor2.setSetpoint(
        leftMotor2.PIDMode.kPosition,
        left.position,
        m_feedforward.calculate(left.velocity));
    rightMotor2.setSetpoint(
        ExampleSmartMotorController.PIDMode.kPosition,
        right.position,
        m_feedforward.calculate(right.velocity));*/

    SparkMaxPIDController leftPIDController = leftMotor2.getPIDController();
    SparkMaxPIDController rightPIDController = rightMotor2.getPIDController();

    //leftPIDController.setP(5e-5);

    //leftPIDController.setI(1e-6);


    leftPIDController.setFF(0.0005);
    rightPIDController.setFF(0.0005);

    leftPIDController.setSmartMotionMaxVelocity(2000, 0);
    leftPIDController.setSmartMotionMaxAccel(1500, 0);
    rightPIDController.setSmartMotionMaxVelocity(2000, 0);
    rightPIDController.setSmartMotionMaxAccel(1500, 0);
    
    leftPIDController.setReference(left.position, CANSparkMax.ControlType.kSmartMotion);
    rightPIDController.setReference(-right.position, CANSparkMax.ControlType.kSmartMotion);
    

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

  public double getAvrageEncoderDistance() {
    return (getLeftEncoderDistance()+getRightEncoderDistance()/2);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(mGyro.getAngle());
  }

  public void setSpeed(final double leftSpeed, final double rightSpeed) {
    m_leftMotors.set(leftSpeed);
    m_rightMotors.set(rightSpeed);
  }

  /** Resets the drive encoders. */
  public void resetEncoders() {
    m_encoderL.setPosition(0.0);
    m_encoderR.setPosition(0.0);
  }

  //Resets the Gyro
  public void resetGyro() {
    mGyro.reset();  
  }

  //Drives a distance
  

     /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    public void logToDashboard() {
      // SmartDashboard.putBoolean("Turn Dampening", getTurnDampening());
      SmartDashboard.putNumber("Drive/Gyro Angle", mGyro.getAngle());
      SmartDashboard.putNumber("Drive/Gyro Pitch", mGyro.getPitch());
      SmartDashboard.putNumber("Drive/Gyro Roll", mGyro.getRoll());
      SmartDashboard.putBoolean("Drive/Gyro Connected", mGyro.isConnected());
      //SmartDashboard.putNumber("Drive/Left Motors Speed Percent", getLeftDriveSpeedPercent());
      //SmartDashboard.putNumber("Drive/Right Motors Speed Percent", getRightDriveSpeedPercent());
      //SmartDashboard.putNumber("Drive/Degree Rotation From Start", getRotationInDegrees());
      //SmartDashboard.putNumber("Drive/Translation On X Since Start", getTranslationX());
      //SmartDashboard.putNumber("Drive/Translation on Y Since Start", getTranslationY());
      SmartDashboard.putNumber("Drive/Left Wheel Speed in Inches per Second", m_encoderL.getVelocity());
      SmartDashboard.putNumber("Drive/Right Wheel Speed in Inches per Second", m_encoderR.getVelocity());
      SmartDashboard.putNumber("Drive/Left Wheel Distance in Inches", m_encoderL.getPosition());
      SmartDashboard.putNumber("Drive/Right Wheel Distance in Inches", m_encoderR.getPosition());
      SmartDashboard.putNumber("Drive/Left Encoder Position Conversion Value", m_encoderL.getPositionConversionFactor());
      //SmartDashboard.putNumber("Drive/X Translation", mOdometry.getPoseMeters().getTranslation().getX());
      //SmartDashboard.putNumber("Drive/Y Translation", mOdometry.getPoseMeters().getTranslation().getY());
      //SmartDashboard.putNumber("Drive/Pose Angle", mOdometry.getPoseMeters().getRotation().getDegrees());
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
  
      // TOO extract it to its own command, then call that wherever
      //These methods define movement according to tank and arcade drive systems as allowed for by the differential drive object. 
      // mDifferentialDrive.arcadeDrive(RobotContainer.mDriverController.getY(Hand.kLeft), RobotContainer.mDriverController.getX(Hand.kRight), false);
    
      //mOdometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
      //mInvertedOdometry.update(getHeading(), -1.0 * getRightDistanceMeters(), -1.0 * getLeftDistanceMeters());
  
      // mOdometry.update(getHeading(), getDifferentialDriveSpeed());
      // mInvertedOdometry.update(getHeading(), getInvertedDifferentialDriveSpeed());
      logToDashboard();
    }

}
