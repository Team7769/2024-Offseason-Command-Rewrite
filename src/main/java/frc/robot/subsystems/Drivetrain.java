package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.utilities.OneDimensionalLookup;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule _frontLeftModule;
    private final SwerveModule _frontRightModule;
    private final SwerveModule _backLeftModule;
    private final SwerveModule _backRightModule;

    private final SwerveDrivePoseEstimator _drivePoseEstimator;

    SwerveModulePosition[] _modulePositions = new SwerveModulePosition[4];

    private double _gyroOffset = 0.0;

    private final Pigeon2 _gyro = new Pigeon2(Constants.DrivetrainConstants.kPigeonId);
    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private SwerveModuleState[] _targetModuleStates = new SwerveModuleState[4];

    private final Field2d m_field = new Field2d();
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(1.75, 0, 0), // Translation constants
            new PIDConstants(1.5, 0, 0), // Rotation constants
            4.5,
            new Translation2d(Constants.DrivetrainConstants.DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, Constants.DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0).getNorm(), // Drive
                                                                                                                 // base
                                                                                                                 // radius
                                                                                                                 // (distance
                                                                                                                 // from
                                                                                                                 // center
                                                                                                                 // to
                                                                                                                 // furthest
                                                                                                                 // module)
            new ReplanningConfig());

    public enum DrivetrainState {
        IDLE,
        OPEN_LOOP,
        TRAJECTORY_FOLLOW,
        TARGET_FOLLOW
    }

    private static class PeriodicIO {
        double VxCmd;
        double VyCmd;
        double WzCmd;
    }

    private final PeriodicIO periodicIO = new PeriodicIO();

    private DrivetrainState _currentState = DrivetrainState.OPEN_LOOP;
    private DrivetrainState _previousState = DrivetrainState.IDLE;
    private Translation2d _target = new Translation2d();
    private boolean _isFollowingFront = false;

    private final CommandXboxController _driverController;

    public Drivetrain(CommandXboxController driverController) {

        _driverController = driverController;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        _frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Front Left Module",
                        BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.DrivetrainConstants.kFrontLeftDriveId)
                .withSteerMotor(MotorType.NEO, Constants.DrivetrainConstants.kFrontLeftSteerId)
                .withSteerEncoderPort(Constants.DrivetrainConstants.kFrontLeftSteerEncoderId)
                .withSteerOffset(Constants.DrivetrainConstants.kFrontLeftEncoderOffset)
                .build();

        _frontRightModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Front Right Module",
                        BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.DrivetrainConstants.kFrontRightDriveId)
                .withSteerMotor(MotorType.NEO, Constants.DrivetrainConstants.kFrontRightSteerId)
                .withSteerEncoderPort(Constants.DrivetrainConstants.kFrontRightSteerEncoderId)
                .withSteerOffset(Constants.DrivetrainConstants.kFrontRightEncoderOffset)
                .build();

        _backLeftModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Back Left Module",
                        BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.DrivetrainConstants.kBackLeftDriveId)
                .withSteerMotor(MotorType.NEO, Constants.DrivetrainConstants.kBackLeftSteerId)
                .withSteerEncoderPort(Constants.DrivetrainConstants.kBackLeftSteerEncoderId)
                .withSteerOffset(Constants.DrivetrainConstants.kBackLeftEncoderOffset)
                .build();

        _backRightModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Back Right Module",
                        BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.FALCON, Constants.DrivetrainConstants.kBackRightDriveId)
                .withSteerMotor(MotorType.NEO, Constants.DrivetrainConstants.kBackRightSteerId)
                .withSteerEncoderPort(Constants.DrivetrainConstants.kBackRightSteerEncoderId)
                .withSteerOffset(Constants.DrivetrainConstants.kBackRightEncoderOffset)
                .build();

        _drivePoseEstimator = new SwerveDrivePoseEstimator(
                Constants.DrivetrainConstants._kinematics,
                getGyroRotation(),
                new SwerveModulePosition[] {
                        _frontLeftModule.getPosition(),
                        _frontRightModule.getPosition(),
                        _backLeftModule.getPosition(),
                        _backRightModule.getPosition()
                },
                new Pose2d());

        AutoBuilder.configureHolonomic(this::getPose, this::setStartingPose, this::getSpeeds, this::setTrajectoryFollowModuleTargets,
                pathFollowerConfig, this::isRedAlliance, this);

        PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", m_field);
        
        var stateLayout = tab.getLayout("State", BuiltInLayouts.kList);
        stateLayout.addString("Drivetrain: Current State", this::getCurrentState);
        stateLayout.addString("Drivetrain: Previous State", this::getPreviousState);

        var fieldLocationLayout = tab.getLayout("Field Locations", BuiltInLayouts.kList);
        fieldLocationLayout.addNumber("Distance to Speaker", this::getDistanceToSpeaker);
        fieldLocationLayout.addNumber("Distance to Zone", this::getDistanceToZone);
        fieldLocationLayout.addNumber("Angle to Speaker", this::getAngleToSpeaker);
        fieldLocationLayout.addNumber("Angle to Zone", this::getAngleToZone);
    }

    private double getDistanceToSpeaker()
    {
        return getDistanceToTarget(Constants.FieldConstants.kSpeaker);
    }

    private double getDistanceToZone()
    {
        return getDistanceToTarget(Constants.FieldConstants.kZone);
    }

    private double getAngleToSpeaker()
    {
        return getAngleToTarget(Constants.FieldConstants.kSpeaker, false);
    }

    private double getAngleToZone()
    {
        return getAngleToTarget(Constants.FieldConstants.kZone, false);
    }

    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }

    public void updateOdometry() {
        var pose = _drivePoseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                getGyroRotation(),
                getModulePositions());

        m_field.setRobotPose(pose);
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.DrivetrainConstants._kinematics.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getGyroRotation() {
        // return rotation2d with first method
        return _gyro.getRotation2d();
    }

    public Rotation2d getGyroRotationWithOffset() {
        // return rotation2d + offset with second method
        return Rotation2d.fromDegrees(_gyro.getRotation2d().getDegrees() +
                _gyroOffset);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                _frontLeftModule.getState(),
                _frontRightModule.getState(),
                _backLeftModule.getState(),
                _backRightModule.getState()
        };
    }

    private void setModuleStates(SwerveModuleState[] moduleStates) {
        // set voltage to deliver to motors and angle to rotate wheel to
        _frontLeftModule.set(moduleStates[0].speedMetersPerSecond /
                Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
                12.0,
                moduleStates[0].angle.getRadians());

        _frontRightModule.set(moduleStates[1].speedMetersPerSecond /
                Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
                12.0,
                moduleStates[1].angle.getRadians());

        _backLeftModule.set(moduleStates[2].speedMetersPerSecond /
                Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
                12.0,
                moduleStates[2].angle.getRadians());

        _backRightModule.set(moduleStates[3].speedMetersPerSecond /
                Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
                12.0,
                moduleStates[3].angle.getRadians());
    }

    public SwerveModulePosition[] getModulePositions() {
        _modulePositions[0] = _frontLeftModule.getPosition();
        _modulePositions[1] = _frontRightModule.getPosition();
        _modulePositions[2] = _backLeftModule.getPosition();
        _modulePositions[3] = _backRightModule.getPosition();
        return _modulePositions;
    }

    public void fieldOrientedDrive(double translationX, double translationY, double rotationZ) {
        var invert = isRedAlliance() ? -1 : 1;

        _chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                invert * translationX * Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                invert * translationY * Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                rotationZ * Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_PER_SECOND,
                getGyroRotationWithOffset());

        drive(_chassisSpeeds);
    }

    /**
     * Method that takes a ChassisSpeeds object and sets each swerve module to it's
     * required state (position, speed, etc) also stores the last modual states
     * applied.
     * 
     * @param chassisSpeeds A variable for a ChasisSpeeds object hold X, Y, and
     *                      rotational velocity as well as the 2D rotation of the
     *                      robot.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        // Sets all the modules to their proper states
        var moduleStates = Constants.DrivetrainConstants._kinematics
                .toSwerveModuleStates(chassisSpeeds);

        // normalize speed based on max velocity meters
        SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates,
                Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

        setModuleStates(moduleStates);
        _chassisSpeeds = chassisSpeeds;
    }

    public void setTrajectoryFollowModuleTargets(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        _targetModuleStates = Constants.DrivetrainConstants._kinematics.toSwerveModuleStates(targetSpeeds);
    }

    public void setStartingPose(Pose2d startingPose) {
        _gyroOffset = startingPose.getRotation().getDegrees();
        SmartDashboard.putNumber("startingPoseRotation", startingPose.getRotation().getDegrees());
        _drivePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), startingPose);
    }

    public Pose2d getPose() {
        return _drivePoseEstimator.getEstimatedPosition();
    }

    public void setTarget(Translation2d targetPosition, boolean isFollowingFront) {
        _target = targetPosition;
        _isFollowingFront = isFollowingFront;
    }

    @Override
    public void periodic() {
        this.periodicIO.VxCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftY());

        // The Y translation will be the horizontal value of the left driver joystick
        this.periodicIO.VyCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftX());

        // The rotation will be the horizontal value of the right driver joystick
        this.periodicIO.WzCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.RotAxis_inputBreakpoints, Constants.DrivetrainConstants.RotAxis_outputTable, _driverController.getRightX());

        updateOdometry();
        handleCurrentState();
    }

    private String getCurrentState() {
        return _currentState.name();
    }
    
    private String getPreviousState() {
        return _previousState.name();
    }

    public void setWantedState(DrivetrainState state) {
        if (state != _currentState) {
            _previousState = _currentState;
            _currentState = state;
        }
    }

    private void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                handleIdle();
                break;
            case OPEN_LOOP:
                handleOpenLoop();
                break;
            case TRAJECTORY_FOLLOW:
                handleTrajectoryFollow();
                break;
            case TARGET_FOLLOW:
                handleTargetFollow();
                break;
            default:
                drive(new ChassisSpeeds());
                break;
        }
    }

    private void handleIdle() {
        drive(new ChassisSpeeds());
    }

    private void handleOpenLoop() {
        fieldOrientedDrive(this.periodicIO.VxCmd, this.periodicIO.VyCmd, this.periodicIO.WzCmd);
    }

    private void handleTrajectoryFollow() {
        setModuleStates(_targetModuleStates);
    }

    private void handleTargetFollow() {
        var rotation = getAngleToTarget(_target, _isFollowingFront) / 105;
        fieldOrientedDrive(this.periodicIO.VxCmd, this.periodicIO.VyCmd, rotation);
    }

    public double getAngleToTarget(Translation2d target, boolean isFollowingFront) {
        return target
            .minus(getPose().getTranslation())
            .getAngle()
            .plus(Rotation2d.fromDegrees(isFollowingFront ? 0 : 180))
            .minus(getGyroRotationWithOffset())
            .getDegrees();
    }

    public double getDistanceToTarget(Translation2d target) {
        return getPose()
            .getTranslation()
            .getDistance(target);
    }

    public double getRotationDifference(double angle) {
        return Rotation2d
            .fromDegrees(angle)
            .minus(getGyroRotationWithOffset())
            .getDegrees();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
