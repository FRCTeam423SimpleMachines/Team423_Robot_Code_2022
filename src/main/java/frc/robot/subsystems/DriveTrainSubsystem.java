package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;


public class DriveTrainSubsystem extends SubsystemBase{

    // The motors on the left side of the drive.
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
        new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless), new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless));

    // The motors on the right side of the drive.
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
        new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless), new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless));

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

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
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }


}
