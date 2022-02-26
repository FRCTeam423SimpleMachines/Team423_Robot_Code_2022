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
    
        public static final int kEncoderCPR = 42;
        public static final double kWheelDiameterInches = 4;
        public static final double kWheelGearRatio = 10.7;
        public static final double kEncoderDistancePerPulse =
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR /(double) kWheelGearRatio;

        public static final double TURN_TOLERANCE_DEG = 5;
        public static final double TURN_RATE_TOLERANCE_DEG_PER_S = 10; // degrees per second  

        public static final double MAX_TURN_RATE_DEG_PER_S = 100;
        public static final double MAX_TURN_ACCELERATION_DEG_PER_S_SQUARED = 300;

        public static final double DRIVE_TOLERANCE_IN = 2;
        public static final double DRIVE_RATE_TOLERANCE_IN_PER_S = 4; // inches per second  

        public static final double MAX_DRIVE_RATE_IN_PER_S = 2700;
        public static final double MAX_DRIVE_ACCELERATION_IN_PER_S_SQUARED = 300;
        
        //Values will need to be adjusted
        public static final double PROFILED_TURN_P = 0.1;
        public static final double PROFILED_TURN_I = 0.0;
        public static final double PROFILED_TURN_D = 0.009;

        public static final double PROFILED_DRIVE_P = 0.1;
        public static final double PROFILED_DRIVE_I = 0.0;
        public static final double PROFILED_DRIVE_D = 0.009;
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
