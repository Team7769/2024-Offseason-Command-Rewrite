// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
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
    public static final Transform3d[] kPhotonCameraOffsets = {new Transform3d(), new Transform3d()}; //TODO: Put in the correct offsets
    public static final String[] kLimelightNames = {"limelight-pose", "limelight-detect"};
    public static final String[] kPhotonCameraNames = {"Arducam_Left", "Arducam_Right"};
    public static final String kTargeterLimelightName = "";
  }

  public static class DrivetrainConstants {
        /*  TODO: this entire section was copied from the constants of our generated swerve
        * I took all of it over because all of it was used we need to look this over later 
        * to see if we need all off it and if it conflicts with otehr constant
        */
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 150.0;

    // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 4.73;
    public static final double MaxAngularRate = 1.5 * Math.PI;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 12.8;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = false;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "DriveCANivore";
    public static final int kPigeonId = 14;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants SwerveConstants = new SwerveDrivetrainConstants()
            .withCANbusName(kCANbusName)
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    public static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
        //     .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
        .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withCANcoderInitialConfigs(cancoderInitialConfigs);


    // Front Left
    public static final int kFrontLeftDriveMotorId = 2;
    public static final int kFrontLeftSteerMotorId = 3;
    public static final int kFrontLeftEncoderId = 4;
    public static final double kFrontLeftEncoderOffset = -0.91259765625;
//     public static final double kFrontLeftEncoderOffset = 179.087402;

    public static final double kFrontLeftXPosInches = 9;
    public static final double kFrontLeftYPosInches = 9;

    // Front Right
    public static final int kFrontRightDriveMotorId = 5;
    public static final int kFrontRightSteerMotorId = 6;
    public static final int kFrontRightEncoderId = 7;
    public static final double kFrontRightEncoderOffset = -0.5576171875;

    public static final double kFrontRightXPosInches = 9;
    public static final double kFrontRightYPosInches = -9;

    // Back Left
    public static final int kBackLeftDriveMotorId = 8;
    public static final int kBackLeftSteerMotorId = 9;
    public static final int kBackLeftEncoderId = 10;
    public static final double kBackLeftEncoderOffset = -0.664794921875;

    private static final double kBackLeftXPosInches = -9;
    private static final double kBackLeftYPosInches = 9;

    // Back Right
    public static final int kBackRightDriveMotorId = 11;
    public static final int kBackRightSteerMotorId = 12;
    public static final int kBackRightEncoderId = 13;
    public static final double kBackRightEncoderOffset = -0.74267578125;

    private static final double kBackRightXPosInches = -9;
    private static final double kBackRightYPosInches = -9;


    public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), true);
    public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);
    public static final SwerveModuleConstants[] modules = {FrontLeft, FrontRight, BackLeft, BackRight};

        // this is where the copied section ends

    public static final double MAX_VOLTAGE = 12.0;
    
    public static final double MAX_MODULE_SPEED = 5.3;

    public static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0.457;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.457;
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
            kSpeedAt12VoltsMps,
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
