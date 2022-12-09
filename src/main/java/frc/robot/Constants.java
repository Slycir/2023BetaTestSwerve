// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final String kRotUnits = "degrees";
    public static final String kDisUnits = "meters";

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
    
        public static final double kEncoderResolution = 4096;
        public static final double kEncoderDistancePerPulse =
            (MeasurementConstants.kWheelDiameterMeters * Math.PI) / kEncoderResolution;
    }

    public static final class MeasurementConstants {
        // This is based on the CAD model
        public static final double kTrackWidthMeters = 0.629; // 24.75 inches - distance between left and right wheels
        public static final double kWheelBaseMeters = 0.629; // 24.75 inches - distance between front and back wheels
        public static final double kWheelDiameterMeters = 0.10033; // 4 inches - diameter of the wheels
    }
    
    public static final class OIConstants {
        public static final int kDriverControllerID = 0;
    }
    
    public static final class DriveConstants {
        public static final double kFrictionFactor = 1.0;
        
        public static final double kDriveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // Wheel revolutions per motor revolution`
        public static final double kSteerReduction = (14.0 / 50.0) * (10.0 / 60.0); // Module revolutions per motor revolution
        
        public static final double kDriveDeadband = 0.02;

        public static final double kMaxSpeedMetersPerSecond = 5880 / 60.0 *
            kDriveReduction *
            MeasurementConstants.kWheelDiameterMeters * Math.PI; // ~ 4.6 m/s
        public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /
            Math.hypot(MeasurementConstants.kTrackWidthMeters / 2.0, MeasurementConstants.kWheelBaseMeters / 2.0); 
        public static final double kSpeedFactor = 1.0; // Used to scale the speed of the robot
        public static final double kMaxVoltage = 12;
        public static final int kDriveCurrentLimit = 80;
        public static final int kSteerCurrentLimit = 20;
        
        public static final double kFrontLeftEncoderOffset = 0; // Must de degrees
        public static final double kBackLeftEncoderOffset = 0; // Must de degrees
        public static final double kFrontRightEncoderOffset = 0; // Must de degrees
        public static final double kBackRightEncoderOffset = 0; // Must de degrees

        // TODO: Tune PID values
        public static final double kSteerP = 1.0;
        public static final double kSteerI = 0.0; // Used in module control
        public static final double kSteerD = 0.1;

        public static final double kDriveP = 5.0;
        public static final double kDriveI = 0.0; // Used in pose control
        public static final double kDriveD = 0.0;

        public static final double kTurnP = 0.5;
        public static final double kTurnI = 0.0; // Used in pose control
        public static final double kTurnD = 0.0;

        public static final double kAccelerationSeconds = 0.5; // 0.5 seconds to reach full speed
    }

    public static HashMap<String, Command> getAutonomousEvents() {
        HashMap<String, Command> events = new HashMap<String, Command>();
        // events.put("Event Name", new EventCommand());
        events.put("PrintTest", new InstantCommand(() -> System.out.println("Test")));
        return events;
    }

    public static Pose2d getApriltagPose(int tagID) {
        switch (tagID) {
            case 0:
                return new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
            default:
                return new Pose2d();
        }
    }
}

