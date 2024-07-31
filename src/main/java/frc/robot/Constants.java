// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.utilities.GeometryUtil;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class FieldConstants {
    public static final double kFieldLength = 16.54;
    public static final double kFieldWidth = 8.21;

    public static final Translation2d kBlueSpeaker = new Translation2d(
        0,
        5.5
    );

    public static final Translation2d kRedSpeaker = GeometryUtil
        .mirrorTranslationForRedAlliance(kBlueSpeaker);

    public static final Translation2d kBlueZone = new Translation2d(
        1.3,
        7.0
    );

    public static final Translation2d kRedZone = GeometryUtil
        .mirrorTranslationForRedAlliance(kBlueZone);
  }

  public static class VisionConstants {
    public static final Transform3d[] kRobotCamOffsets = {};
    public static final String[] kLimelightNames = {};
    public static final String[] kPhotonCameraNames = {};
    public static final String kTargeterLimelightName = "";
  }

  public static class DrivetrainConstants {    
    public static final int kFrontLeftDriveId = 2;
    public static final int kFrontLeftSteerId = 3;
    public static final int kFrontLeftSteerEncoderId = 4;

    public static final int kFrontRightDriveId = 5;
    public static final int kFrontRightSteerId = 6;
    public static final int kFrontRightSteerEncoderId = 7;

    public static final int kBackLeftDriveId = 8;
    public static final int kBackLeftSteerId = 9;
    public static final int kBackLeftSteerEncoderId = 10;

    public static final int kBackRightDriveId = 11;
    public static final int kBackRightSteerId = 12;
    public static final int kBackRightSteerEncoderId = 13;
    public static final int kPigeonId = 14;
    
    public static final double kFrontLeftEncoderOffset = -Math.toRadians(85.86914);
    public static final double kFrontRightEncoderOffset = -Math.toRadians(348.57421);
    public static final double kBackLeftEncoderOffset = -Math.toRadians(219.55078);
    public static final double kBackRightEncoderOffset = -Math.toRadians(295.40039);

    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 /
            60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L2.getWheelDiameter() *
            Math.PI;
    public static final double MAX_MODULE_SPEED = 5.3;

    public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.52705;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.52705;
    public static final double MAX_ANGULAR_VELOCITY_PER_SECOND = 3 * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_PER_SECOND_SQUARED = MAX_ANGULAR_VELOCITY_PER_SECOND *
            MAX_ANGULAR_VELOCITY_PER_SECOND;

    public static final double DRIVE_BASE_RADIUS = 0.3698875;

    // setting up kinematics
    public static final SwerveDriveKinematics _kinematics = new SwerveDriveKinematics(
            // Front Left
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),

            // Front Right
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),

            // Back Left
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),

            // Back Right
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(1.75, 0, 0), // Translation constants
            new PIDConstants(1.5, 0, 0), // Rotation constants
            MAX_MODULE_SPEED,
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0).getNorm(), // Drive
                                                                                                                 // base
                                                                                                                 // radius
                                                                                                                 // (distance
                                                                                                                 // from
                                                                                                                 // center
                                                                                                                 // to
                                                                                                                 // furthest
                                                                                                                 // module)
            new ReplanningConfig());
    // Deadband
    public static final double kDeadband = 0.15;
    public static final double[] XY_Axis_inputBreakpoints = { -1, -0.9, -0.85, -0.7, -0.6, -0.5, -0.2, -0.12, 0.12, 0.2,
            0.5, 0.6, 0.7, 0.85, .9, 1 };
    public static final double[] XY_Axis_outputTable = { -1.0, -.7, -0.6, -0.4, -0.3, -0.2, -0.05, 0, 0, 0.05, 0.2, 0.3,
            0.4, 0.6, .7, 1.0 };
    public static final double[] RotAxis_inputBreakpoints = { -1, -.9, -0.85, -0.7, -0.6, -0.5, -0.2, -0.12, 0.12, 0.2,
            0.5, 0.6, 0.7, 0.85, .9, 1 };
    public static final double[] RotAxis_outputTable = { -1.0, -.7, -0.6, -0.4, -0.3, -0.2, -0.05, 0, 0, 0.05, 0.2, 0.3,
            0.4, 0.6, .7, 1.0 };
  }
}
