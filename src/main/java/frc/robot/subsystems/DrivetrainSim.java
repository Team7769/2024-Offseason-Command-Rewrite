package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.enums.DrivetrainState;
import frc.robot.simulation.*;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.OneDimensionalLookup;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DrivetrainSim extends SubsystemBase implements IDrivetrain {    
    private final SimSwerveModule _frontLeftModule;
    private final SimSwerveModule _frontRightModule;
    private final SimSwerveModule _backLeftModule;
    private final SimSwerveModule _backRightModule;
    private final SimGyro _gyro;
    private final Field2d _field = new Field2d();
    SwerveModulePosition[] _modulePositions = new SwerveModulePosition[4];
    private final SwerveDrivePoseEstimator _drivePoseEstimator;
    private double _gyroOffset = 0.0;

    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private SwerveModuleState[] _targetModuleStates = new SwerveModuleState[4];    

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

    public DrivetrainSim(CommandXboxController driverController) {
        
        _driverController = driverController;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        _frontLeftModule = new SimSwerveModule();
        _frontRightModule = new SimSwerveModule();
        _backLeftModule = new SimSwerveModule();
        _backRightModule = new SimSwerveModule();
        _gyro = new SimGyro();

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
                Constants.DrivetrainConstants.pathFollowerConfig, GeometryUtil::isRedAlliance, this);

        PathPlannerLogging.setLogActivePathCallback((poses) -> _field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", _field);
        
        var stateLayout = tab.getLayout("State", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
        stateLayout.addString("Drivetrain: Current State", this::getCurrentState);
        stateLayout.addString("Drivetrain: Previous State", this::getPreviousState);

        var fieldLocationLayout = tab.getLayout("Field Locations", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0);
        fieldLocationLayout.addNumber("Distance to Speaker", this::getDistanceToSpeaker);
        fieldLocationLayout.addNumber("Distance to Zone", this::getDistanceToZone);
        fieldLocationLayout.addNumber("Angle to Speaker", this::getAngleToSpeaker);
        fieldLocationLayout.addNumber("Angle to Zone", this::getAngleToZone);
    }

    //#region Suppliers
    private double getDistanceToSpeaker()
    {
        return GeometryUtil.getDistanceToTarget(GeometryUtil.kSpeaker, this::getPose);
    }

    private double getDistanceToZone()
    {
        return GeometryUtil.getDistanceToTarget(GeometryUtil.kZone, this::getPose);
    }

    private double getAngleToSpeaker()
    {
        return GeometryUtil.getAngleToTarget(GeometryUtil.kSpeaker, this::getPose, _isFollowingFront);
    }

    private double getAngleToZone()
    {
        return GeometryUtil.getAngleToTarget(GeometryUtil.kZone, this::getPose, _isFollowingFront);
    }

    private ChassisSpeeds getSpeeds() {
        return Constants.DrivetrainConstants._kinematics.toChassisSpeeds(getModuleStates());
    }

    private Rotation2d getGyroRotation() {
        // return rotation2d with first method
        return _gyro.getRotation2d();
    }

    private Rotation2d getGyroRotationWithOffset() {
        // return rotation2d + offset with second method
        return Rotation2d.fromDegrees(_gyro.getRotation2d().getDegrees() +
                _gyroOffset);
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                _frontLeftModule.getState(),
                _frontRightModule.getState(),
                _backLeftModule.getState(),
                _backRightModule.getState()
        };
    }

    private void setModuleStates(SwerveModuleState[] moduleStates) {
        _frontLeftModule.setTargetState(moduleStates[0]);
        _frontRightModule.setTargetState(moduleStates[1]);
        _backLeftModule.setTargetState(moduleStates[2]);
        _backRightModule.setTargetState(moduleStates[3]);

        _targetModuleStates = moduleStates;
    }

    private SwerveModulePosition[] getModulePositions() {
        _modulePositions[0] = _frontLeftModule.getPosition();
        _modulePositions[1] = _frontRightModule.getPosition();
        _modulePositions[2] = _backLeftModule.getPosition();
        _modulePositions[3] = _backRightModule.getPosition();
        return _modulePositions;
    }    

    private void setTrajectoryFollowModuleTargets(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        _targetModuleStates = Constants.DrivetrainConstants._kinematics.toSwerveModuleStates(targetSpeeds);
    }

    private void setStartingPose(Pose2d startingPose) {
        _gyroOffset = startingPose.getRotation().getDegrees();
        _drivePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), startingPose);
    }

    private Pose2d getPose() {
        return _drivePoseEstimator.getEstimatedPosition();
    }

    private String getCurrentState() {
        return _currentState.name();
    }
    
    private String getPreviousState() {
        return _previousState.name();
    }

    //#endregion

    private void updateOdometry() {
        var pose = _drivePoseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                getGyroRotation(),
                getModulePositions());

        _field.setRobotPose(pose);
    }

    private void fieldOrientedDrive(double translationX, double translationY, double rotationZ) {
        var invert = GeometryUtil.isRedAlliance() ? -1 : 1;

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
    private void drive(ChassisSpeeds chassisSpeeds) {
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

    @Override
    public void periodic() {
        _gyro.updateRotation(getSpeeds().omegaRadiansPerSecond);
        this.periodicIO.VxCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftY());

        // The Y translation will be the horizontal value of the left driver joystick
        this.periodicIO.VyCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftX());

        // The rotation will be the horizontal value of the right driver joystick
        this.periodicIO.WzCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.RotAxis_inputBreakpoints, Constants.DrivetrainConstants.RotAxis_outputTable, _driverController.getRightX());

        updateOdometry();
        handleCurrentState();
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
        var rotation = GeometryUtil.getAngleToTarget(_target, this::getPose, _isFollowingFront) / 50;
        fieldOrientedDrive(this.periodicIO.VxCmd, this.periodicIO.VyCmd, rotation);
    }

    //#region Commands
    public SequentialCommandGroup targetSpeaker(Supplier<Boolean> isRedAlliance) {
        return new SequentialCommandGroup(new InstantCommand(() -> {
            _target = isRedAlliance.get() ? Constants.FieldConstants.kRedSpeaker : Constants.FieldConstants.kBlueSpeaker;
            _isFollowingFront = false;
        }, this), setWantedState(DrivetrainState.TARGET_FOLLOW));
    }

    public SequentialCommandGroup targetZone(Supplier<Boolean> isRedAlliance) {
        return new SequentialCommandGroup(new InstantCommand(() -> {
            _target = isRedAlliance.get() ? Constants.FieldConstants.kRedZone : Constants.FieldConstants.kBlueZone;
            _isFollowingFront = false;
        }, this), setWantedState(DrivetrainState.TARGET_FOLLOW));
    }

    public InstantCommand setWantedState(DrivetrainState state) {
        return new InstantCommand(() -> {
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;
            }
        }, this);
    }
    //#endregion

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
