package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.enums.DrivetrainState;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.OneDimensionalLookup;

public class Drivetrain extends CommandSwerveDrivetrain implements IDrivetrain {
    // Request to apply to the drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest idle = new SwerveRequest.Idle();

    // Pose Estimator
    // private final SwerveDrivePoseEstimator _drivePoseEstimator;

    // Gyro (Pigeon)
    // private final Pigeon2 _gyro;

    private final Field2d m_field = new Field2d();
    private final PeriodicIO periodicIO = new PeriodicIO();

    private final Vision m_vision = new Vision();

    // private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    // private SwerveModuleState[] _targetModuleStates = new SwerveModuleState[4];
    // SwerveModulePosition[] _modulePositions = new SwerveModulePosition[4];
    private DrivetrainState _currentState = DrivetrainState.OPEN_LOOP;
    private DrivetrainState _previousState = DrivetrainState.IDLE;
    private Translation2d _target = new Translation2d();
    private boolean _isFollowingFront = false;

    private static class PeriodicIO {
        double VxCmd;
        double VyCmd;
        double WzCmd;
    }

    // Dependencies
    private final CommandXboxController _driverController;
    public Drivetrain(CommandXboxController driverController) {
        super(DrivetrainConstants.SwerveConstants, DrivetrainConstants.modules);
        _driverController = driverController;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        // TODO: See if we still need an autobuilder
        // AutoBuilder.configureHolonomic(this::getPose, this::setStartingPose, this::getSpeeds, this::setTrajectoryFollowModuleTargets,
        // Constants.DrivetrainConstants.pathFollowerConfig, GeometryUtil::isRedAlliance, this);

        PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", m_field);
        
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
    // TODO: See if we still need this function
    // private void setTrajectoryFollowModuleTargets(ChassisSpeeds robotRelativeSpeeds) {
    //     ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    //     _targetModuleStates = Constants.DrivetrainConstants._kinematics.toSwerveModuleStates(targetSpeeds);
    // }

    private void setStartingPose(Pose2d startingPose) {
        this.seedFieldRelative(startingPose);
    }

    private Pose2d getPose() {
        return this.m_odometry.getEstimatedPosition();
    }    
     
    private ChassisSpeeds getSpeeds() {
        return Constants.DrivetrainConstants._kinematics.toChassisSpeeds(getModuleStates());
    }

    private Rotation2d getGyroRotation() {
        // return rotation2d with first method
        return this.m_pigeon2.getRotation2d();
    }

    private SwerveModuleState[] getModuleStates() {
        return this.m_moduleStates;

    }
    // TODO: this command seems impossible to preform with the swerve drivetrain
    // private void setModuleStates(SwerveModuleState[] moduleStates) {
    //     // set voltage to deliver to motors and angle to rotate wheel to
    //     _frontLeftModule.set(moduleStates[0].speedMetersPerSecond /
    //             Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
    //             Constants.DrivetrainConstants.MAX_VOLTAGE,
    //             moduleStates[0].angle.getRadians());

    //     _frontRightModule.set(moduleStates[1].speedMetersPerSecond /
    //             Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
    //             Constants.DrivetrainConstants.MAX_VOLTAGE,
    //             moduleStates[1].angle.getRadians());

    //     _backLeftModule.set(moduleStates[2].speedMetersPerSecond /
    //             Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
    //             Constants.DrivetrainConstants.MAX_VOLTAGE,
    //             moduleStates[2].angle.getRadians());

    //     _backRightModule.set(moduleStates[3].speedMetersPerSecond /
    //             Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
    //             Constants.DrivetrainConstants.MAX_VOLTAGE,
    //             moduleStates[3].angle.getRadians());
                
    //     _targetModuleStates = moduleStates;
    // }

    private SwerveModulePosition[] getModulePositions() {
        return this.m_modulePositions;
    }    

    private String getCurrentState() {
        return _currentState.name();
    }
    
    private String getPreviousState() {
        return _previousState.name();
    }
    //#endregion
    
    // private void updateOdometry() {
    //     var pose = _drivePoseEstimator.updateWithTime(
    //             Timer.getFPGATimestamp(),
    //             getGyroRotation(),
    //             getModulePositions());

    //     m_field.setRobotPose(pose);
    // }

    private void fieldOrientedDrive(double translationX, double translationY, double rotationZ) {
        applyRequest(() -> drive.withVelocityX(translationX * DrivetrainConstants.kSpeedAt12VoltsMps)
            .withVelocityY(translationY * DrivetrainConstants.kSpeedAt12VoltsMps)
            .withRotationalRate(rotationZ * DrivetrainConstants.MaxAngularRate));
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
    // private void drive(ChassisSpeeds chassisSpeeds) {
    //     // Sets all the modules to their proper states
    //     var moduleStates = Constants.DrivetrainConstants._kinematics
    //             .toSwerveModuleStates(chassisSpeeds);

    //     // normalize speed based on max velocity meters
    //     SwerveDriveKinematics.desaturateWheelSpeeds(
    //             moduleStates,
    //             Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);

    //     setModuleStates(moduleStates);
    //     _chassisSpeeds = chassisSpeeds;
    // }

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
    public void periodic() {
        this.periodicIO.VxCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftY());

        // The Y translation will be the horizontal value of the left driver joystick
        this.periodicIO.VyCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftX());

        // The rotation will be the horizontal value of the right driver joystick
        this.periodicIO.WzCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.RotAxis_inputBreakpoints, Constants.DrivetrainConstants.RotAxis_outputTable, _driverController.getRightX());

        // updateOdometry();
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
            // case TRAJECTORY_FOLLOW:
            //     handleTrajectoryFollow();
            //     break;
            case TARGET_FOLLOW:
                handleTargetFollow();
                break;
            case NOTE_FOLLOW:
                handleNoteFollow();
            case MOVE_TO_NOTE:
                handleMoveToNote();
            default:
                applyRequest(() -> idle);
                break;

        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {  
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void handleIdle() {
        applyRequest(() -> idle);
    }

    private void handleOpenLoop() {
        fieldOrientedDrive(this.periodicIO.VxCmd, this.periodicIO.VyCmd, this.periodicIO.WzCmd);
    }

    // private void handleTrajectoryFollow() {
    //     setModuleStates(_targetModuleStates);
    // }

    private void handleTargetFollow() {
        var rotation = GeometryUtil.getAngleToTarget(_target, this::getPose, _isFollowingFront) / 50;
        fieldOrientedDrive(this.periodicIO.VxCmd, this.periodicIO.VyCmd, rotation);
    }

    private void handleNoteFollow()
    {
        var rotation = m_vision.getTargetAngle(); 
        fieldOrientedDrive(this.periodicIO.VxCmd, this.periodicIO.VyCmd, rotation);
    }

    private void handleMoveToNote()
    {
        //somehow
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
