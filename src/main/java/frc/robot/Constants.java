// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // CAN IDs are not final
    // They are subject to change when the chassis is built
    public static final class CANConstants {
        public static final int kFrontLeftDriveMotorID = 1;
        public static final int kBackLeftDriveMotorID = 2;
        public static final int kFrontRightDriveMotorID = 3;
        public static final int kBackRightDriveMotorID = 4;
    
        public static final int kFrontLeftSteerMotorID = 5;
        public static final int kBackLeftSteerMotorID = 6;
        public static final int kFrontRightSteerMotorID = 7;
        public static final int kBackRightSteerMotorID = 8;
    
        public static final int kFrontLeftEncoderID = 9;
        public static final int kBackLeftEncoderID = 10;
        public static final int kFrontRightEncoderID = 11;
        public static final int kBackRightEncoderID = 12;
    
        // public static final int kFrontLeftEncoderReversed = 1;
        // public static final int kBackLeftEncoderReversed = 1;
        // public static final int kFrontRightEncoderReversed = 1;
        // public static final int kBackRightEncoderReversed = 1;
    
        public static final double kWheelDiameterMeters = SdsModuleConfigurations.MK4_L2.getWheelDiameter();
        public static final double kEncoderResolution = 4096;
        public static final double kEncoderDistancePerPulse =
            (kWheelDiameterMeters * Math.PI) / kEncoderResolution;
    }

    public static final class MeasurementConstants {
        // This is based on the CAD model
        public static final double kTrackWidthMeters = 0.629; // 24.75 inches - distance between left and right wheels
        public static final double kWheelBaseMeters = 0.629; // 24.75 inches - distance between front and back wheels
    }
    
    public static final class OIConstants {
        public static final int kDriverControllerID = 0;
    }
    
    public static final class DriveConstants {
        public static final double kMaxSpeedMetersPerSecond = 5880 / 60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            CANConstants.kWheelDiameterMeters * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /
            Math.hypot(MeasurementConstants.kTrackWidthMeters / 2.0, MeasurementConstants.kWheelBaseMeters / 2.0);
        public static final double kMaxVoltage = 12;
        
        public static final double kFrontLeftEncoderOffset = 0;
        public static final double kBackLeftEncoderOffset = 0;
        public static final double kFrontRightEncoderOffset = 0;
        public static final double kBackRightEncoderOffset = 0;
    }
}

