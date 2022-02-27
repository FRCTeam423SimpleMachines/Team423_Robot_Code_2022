// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kLeftMotor3Port = 3;
        public static final int kRightMotor1Port = 4;
        public static final int kRightMotor2Port = 5;
        public static final int kRightMotor3Port = 6;
    
        // Might not need these lines
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    
        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

        public static final double TURN_TOLERANCE_DEG = 5;
        public static final double TURN_RATE_TOLERANCE_DEG_PER_S = 10; // degrees per second  

        public static final double MAX_TURN_RATE_DEG_PER_S = 100;
        public static final double MAX_TURN_ACCELERATION_DEG_PER_S_SQUARED = 300;
        
        //Values will need to be adjusted
        public static final double PROFILED_TURN_P = 0.1;
        public static final double PROFILED_TURN_I = 0.0;
        public static final double PROFILED_TURN_D = 0.009;
      }

    public static final class TurretConstants {
        public static final int kTurretMotorPort = 21;

        public static final int TURRET_GEAR_RATIO = 560;
        public static final int ENCODER_CLICKS = 42;
        public static final int TOTAL_CLICKS_PER_ROTAION = ENCODER_CLICKS*TURRET_GEAR_RATIO;
        public static final double CLICKS_PER_DEGREE = (double) TOTAL_CLICKS_PER_ROTAION/360;
    }

    public static final class AutoConstants {
        public static final double kAutoDriveDistanceInches = 60;
        public static final double kAutoBackupDistanceInches = 20;
        public static final double kAutoDriveSpeed = 0.5;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverControllerPort2 = 1;
      }
}
