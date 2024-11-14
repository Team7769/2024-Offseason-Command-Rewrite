package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.enums.DrivetrainState;
import frc.robot.statemachine.StateBasedSubsystem;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.OneDimensionalLookup;
import frc.robot.utilities.VisionMeasurement;

public class SDSDrivetrain extends StateBasedSubsystem<DrivetrainState> implements IDrivetrain {
    // Request to apply to the drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest idle = new SwerveRequest.Idle();
    // this is used to drive with chassis speeds see an example of it in setTrajectoryFollowModuleTargets
    public final SwerveRequest.ApplyChassisSpeeds chassisDrive = new SwerveRequest.ApplyChassisSpeeds();

    private final SwerveModule _frontLeftModule;
    private final SwerveModule _frontRightModule;
    private final SwerveModule _backLeftModule;
    private final SwerveModule _backRightModule;
    private final Pigeon2 m_pigeon2 = new Pigeon2(Constants.DrivetrainConstants.kPigeonId);
    private final SwerveDrivePoseEstimator _drivePoseEstimator;
    private final Field2d m_field = new Field2d();
    private final PeriodicIO periodicIO = new PeriodicIO();
    private boolean hasAppliedOperatorPerspective = false;
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    private SwerveModuleState[] _moduleStates = new SwerveModuleState[4];
    private SwerveModulePosition[] _modulePositions = new SwerveModulePosition[4];
    private Translation2d _target = new Translation2d();
    private boolean _isFollowingFront = false;
    private double targetRotation;

    private static class PeriodicIO {
        double VxCmd;
        double VyCmd;
        double WzCmd;
    }

    // Dependencies
    private final CommandXboxController _driverController;
    private final Vision _vision;
    private ChassisSpeeds followChassisSpeeds = new ChassisSpeeds(0,0,0);
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

    public SDSDrivetrain(CommandXboxController driverController, Vision vision) {
        _driverController = driverController;
        _vision = vision;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        AutoBuilder.configureHolonomic(this::getPose, this::setStartingPose, this::getSpeeds, this::setSpeeds,
        Constants.DrivetrainConstants.pathFollowerConfig, GeometryUtil::isRedAlliance, this);


        PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
        SmartDashboard.putData("Field", m_field);

        _currentState = DrivetrainState.OPEN_LOOP;
        _previousState = DrivetrainState.IDLE;

        _frontLeftModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Left Module",
                                      BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.DrivetrainConstants.kFrontLeftDriveMotorId)
            .withSteerMotor(MotorType.NEO, Constants.DrivetrainConstants.kFrontLeftSteerMotorId)
            .withSteerEncoderPort(Constants.DrivetrainConstants.kFrontLeftEncoderId)
            .withSteerOffset(Constants.DrivetrainConstants.kFrontLeftEncoderOffset)
            .build();

        _frontRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Front Right Module",
                                      BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.DrivetrainConstants.kFrontRightDriveMotorId)
            .withSteerMotor(MotorType.NEO, Constants.DrivetrainConstants.kFrontRightSteerMotorId)
            .withSteerEncoderPort(Constants.DrivetrainConstants.kFrontRightEncoderId)
            .withSteerOffset(Constants.DrivetrainConstants.kFrontRightEncoderOffset)
            .build();

        _backLeftModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Back Left Module",
                                      BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.DrivetrainConstants.kBackLeftDriveMotorId)
            .withSteerMotor(MotorType.NEO, Constants.DrivetrainConstants.kBackLeftSteerMotorId)
            .withSteerEncoderPort(Constants.DrivetrainConstants.kBackLeftEncoderId)
            .withSteerOffset(Constants.DrivetrainConstants.kBackLeftEncoderOffset)
            .build();

        _backRightModule = new MkSwerveModuleBuilder()
            .withLayout(tab.getLayout("Back Right Module",
                                      BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0))
            .withGearRatio(SdsModuleConfigurations.MK4I_L2)
            .withDriveMotor(MotorType.FALCON, Constants.DrivetrainConstants.kBackRightDriveMotorId)
            .withSteerMotor(MotorType.NEO, Constants.DrivetrainConstants.kBackRightSteerMotorId)
            .withSteerEncoderPort(Constants.DrivetrainConstants.kBackRightEncoderId)
            .withSteerOffset(Constants.DrivetrainConstants.kBackRightEncoderOffset)
            .build();

        _drivePoseEstimator = new SwerveDrivePoseEstimator(
            Constants.DrivetrainConstants._kinematics,
            getRotation2d(),
            new SwerveModulePosition[] {
                _frontLeftModule.getPosition(),
                _frontRightModule.getPosition(),
                _backLeftModule.getPosition(),
                _backRightModule.getPosition()
            },
            new Pose2d()
        );
        
        var stateLayout = tab.getLayout("State", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
        stateLayout.addString("Drivetrain: Current State", this::getCurrentStateName);
        stateLayout.addString("Drivetrain: Previous State", this::getPreviousStateName);

        var fieldLocationLayout = tab.getLayout("Field Locations", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0);
        fieldLocationLayout.addNumber("Distance to Speaker", this::getDistanceToSpeaker);
        fieldLocationLayout.addNumber("Distance to Zone", this::getDistanceToZone);
        fieldLocationLayout.addNumber("Angle to Speaker", this::getAngleToSpeaker);
        fieldLocationLayout.addNumber("Angle to Zone", this::getAngleToZone);
        fieldLocationLayout.addNumber("VX", this::getVxCmd);
        fieldLocationLayout.addNumber("VY", this::getVyCmd);
        fieldLocationLayout.addNumber("WZ", this::getWzCmd);
        fieldLocationLayout.addNumber("rotation", this::getDegrees);
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

    private double getAngleToTarget(Translation2d translation) {
        return GeometryUtil.getAngleToTarget(
            translation, this::getPose, _isFollowingFront
        );
    }

    private double getDistanceToTarget(Translation2d translation) {
        return GeometryUtil.getDistanceToTarget(
            translation, this::getPose
        );
    }

    private void setTrajectoryFollowModuleTargets(ChassisSpeeds robotRelativeSpeeds) {
        System.out.println("setTrajectoryFollowModuleTargets");
        followChassisSpeeds = robotRelativeSpeeds;
    }

    private void setStartingPose(Pose2d startingPose) {
        System.out.println("setStartingPose");
        _drivePoseEstimator.resetPosition(getRotation2d(), _modulePositions, startingPose);
    }

    public Pose2d getPose() {
        
        // System.out.println("getPose");
        return _drivePoseEstimator.getEstimatedPosition();
    
    }    

    public SwerveModulePosition[] getModulePositions()
    {
        _modulePositions[0] = _frontLeftModule.getPosition();
        _modulePositions[1] = _frontRightModule.getPosition();
        _modulePositions[2] = _backLeftModule.getPosition();
        _modulePositions[3] = _backRightModule.getPosition();
        return _modulePositions;
    }

    // private SwerveModuleState[] getModuleStates() {
    //     return this.m_moduleStates;
    // }

    private ChassisSpeeds getSpeeds() {
        return Constants.DrivetrainConstants._kinematics.toChassisSpeeds(_moduleStates);
    }

    public InstantCommand resetGyro() {
        return new InstantCommand(() -> m_pigeon2.setYaw(0));
    }

    private double getVxCmd() {
        return this.periodicIO.VxCmd;
    }

    private double getVyCmd() {
        return this.periodicIO.VyCmd;
    }

    private double getWzCmd() {
        return this.periodicIO.WzCmd;
    }

    private double getDegrees() {
        return m_pigeon2.getAngle();
    }

    private Rotation2d getRotation2d() {
        return m_pigeon2.getRotation2d();
    }
    //#endregion

    private void updateOdometry() {
        // _vision.imposeVisionMeasurements(this);
        _drivePoseEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            getRotation2d(),
            new SwerveModulePosition[] {
                _frontLeftModule.getPosition(),
                _frontRightModule.getPosition(),
                _backLeftModule.getPosition(),
                _backRightModule.getPosition()
            }
        );

        ArrayList<VisionMeasurement> visionMeasurements = _vision.getVisionMeasurements(m_pigeon2.getRotation2d());

        for (VisionMeasurement visionMeasurement : visionMeasurements) {
            _drivePoseEstimator.addVisionMeasurement(
                visionMeasurement.pose, visionMeasurement.timestamp
            );
        }

        m_field.setRobotPose(getPose());
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

    @Override
    public InstantCommand setWantedState(DrivetrainState state) {
        return new InstantCommand(() -> {
            System.out.print(state.name());
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;
            }
        }, this);
    }
    //#endregion

    @Override
    public void periodic() {
        // if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        //     DriverStation.getAlliance().ifPresent((allianceColor) -> {
        //         this.setOperatorPerspectiveForward(
        //                 allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
        //                         : BlueAlliancePerspectiveRotation);
        //         hasAppliedOperatorPerspective = true;
        //     });
        // }
        this.periodicIO.VxCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftX());

        // The Y translation will be the horizontal value of the left driver joystick
        this.periodicIO.VyCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftY());

        // The rotation will be the horizontal value of the right driver joystick
        this.periodicIO.WzCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.RotAxis_inputBreakpoints, Constants.DrivetrainConstants.RotAxis_outputTable, _driverController.getRightX());
        targetRotation = GeometryUtil.getAngleToTarget(_target, this::getPose, _isFollowingFront) / 50;

        updateOdometry();
        handleCurrentState().schedule();
    }

    private Command handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                return fieldDrive(0, 0, 0);
            case OPEN_LOOP:
                return fieldDrive(periodicIO.VxCmd, periodicIO.VyCmd, periodicIO.WzCmd);
            case TRAJECTORY_FOLLOW:
                return drive(chassisSpeeds);
                //return applyRequest(() -> chassisDrive.withSpeeds(followChassisSpeeds));
            //     return applyRequest(() -> drive.withVelocityX(this.followChassisSpeeds.vxMetersPerSecond * DrivetrainConstants.kSpeedAt12VoltsMps)
            // .withVelocityY(this.followChassisSpeeds.vyMetersPerSecond * DrivestrainConstants.kSpeedAt12VoltsMps)
            // .withRotationalRate(this.followChassisSpeeds.omegaRadiansPerSecond * DrivetrainConstants.MaxAngularRate));
            // return new InstantCommand();
            case TARGET_FOLLOW:
                return fieldDrive(periodicIO.VxCmd, periodicIO.VyCmd, targetRotation);
            case NOTE_FOLLOW:
                return handleNoteFollow();
            // case MOVE_TO_NOTE:
            //     handleMoveToNote();
            //     break;
            default:
                return fieldDrive(0, 0, 0);

        }
    }

    public void setSpeeds(ChassisSpeeds newSpeeds) {
        chassisSpeeds = newSpeeds;
    }

    // public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {  
    //     return run(() -> this.setControl(requestSupplier.get()));
    // }

    public Command fieldDrive(double x, double y, double z) {
        var alliance = DriverStation.getAlliance();
            var invert = alliance.isPresent() && alliance.get() == Alliance.Red ? -1 : 1; 

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                invert * x * Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
                invert * y * Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND, 
                z * Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_PER_SECOND, 
                m_pigeon2.getRotation2d()
            );
            
        return drive(chassisSpeeds);
    }

    public Command drive(ChassisSpeeds speeds) {
        return run(()-> {
            var moduleStates = Constants
            .DrivetrainConstants._kinematics
            .toSwerveModuleStates(chassisSpeeds);
        
            // normalize speed based on max velocity meters
            SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates,
                Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
            );

            setModuleStates(moduleStates);
            _moduleStates = moduleStates;
            }
        );
    }

    private void setModuleStates(SwerveModuleState[] moduleStates)
    {
        // set voltage to deliver to motors and angle to rotate wheel to
        _frontLeftModule.set(moduleStates[0].speedMetersPerSecond /
                             Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
                             Constants.DrivetrainConstants.MAX_VOLTAGE,
                             moduleStates[0].angle.getRadians());

        _frontRightModule.set(moduleStates[1].speedMetersPerSecond /
                              Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
                             Constants.DrivetrainConstants.MAX_VOLTAGE,
                              moduleStates[1].angle.getRadians());

        _backLeftModule.set(moduleStates[2].speedMetersPerSecond /
                            Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
                            Constants.DrivetrainConstants.MAX_VOLTAGE,
                            moduleStates[2].angle.getRadians());

        _backRightModule.set(moduleStates[3].speedMetersPerSecond /
                             Constants.DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND *
                             Constants.DrivetrainConstants.MAX_VOLTAGE,
                             moduleStates[3].angle.getRadians());
    }

    private Command handleNoteFollow()
    {
        var rotation = _vision.getNoteAngle(); 
        // return applyRequest(() -> drive.withVelocityX(this.periodicIO.VxCmd * DrivetrainConstants.kSpeedAt12VoltsMps)
        //         .withVelocityY(this.periodicIO.VyCmd * DrivetrainConstants.kSpeedAt12VoltsMps)
        //         .withRotationalRate(-rotation * DrivetrainConstants.MaxAngularRate * 0.012));
        return fieldDrive(this.periodicIO.VxCmd, this.periodicIO.VyCmd, -rotation * DrivetrainConstants.MaxAngularRate * 0.001);
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
