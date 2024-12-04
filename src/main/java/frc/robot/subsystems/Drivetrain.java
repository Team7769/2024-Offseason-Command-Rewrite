package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.enums.DrivetrainState;
import frc.robot.utilities.GeometryUtil;
import frc.robot.utilities.OneDimensionalLookup;
import frc.robot.utilities.VisionMeasurement;

public class Drivetrain extends CommandSwerveDrivetrain implements IDrivetrain {
    // Request to apply to the drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final SwerveRequest idle = new SwerveRequest.Idle();
    // this is used to drive with chassis speeds see an example of it in setTrajectoryFollowModuleTargets
    public final SwerveRequest.ApplyChassisSpeeds chassisDrive = new SwerveRequest.ApplyChassisSpeeds();


    private final Field2d m_field = new Field2d();
    private final PeriodicIO periodicIO = new PeriodicIO();
    private boolean hasAppliedOperatorPerspective = false;
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    private DrivetrainState _currentState = DrivetrainState.OPEN_LOOP;
    private DrivetrainState _previousState = DrivetrainState.IDLE;
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

    public Drivetrain(CommandXboxController driverController, Vision vision) {
        super(DrivetrainConstants.SwerveConstants, DrivetrainConstants.modules);

        _driverController = driverController;
        _vision = vision;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        AutoBuilder.configureHolonomic(this::getPose, this::setStartingPose, this::getSpeeds, (speeds) -> this.setControl(chassisDrive.withSpeeds(speeds)),
        Constants.DrivetrainConstants.pathFollowerConfig, GeometryUtil::isRedAlliance, this);


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

    // private double getAngleToTarget(Translation2d translation) {
    //     return GeometryUtil.getAngleToTarget(
    //         translation, this::getPose, _isFollowingFront
    //     );
    // }

    // private double getDistanceToTarget(Translation2d translation) {
    //     return GeometryUtil.getDistanceToTarget(
    //         translation, this::getPose
    //     );
    // }

    // private void setTrajectoryFollowModuleTargets(ChassisSpeeds robotRelativeSpeeds) {
    //     System.out.println("setTrajectoryFollowModuleTargets");
    //     followChassisSpeeds = robotRelativeSpeeds;
    // }

    private void setStartingPose(Pose2d startingPose) {
        System.out.println("setStartingPose");
        this.seedFieldRelative(startingPose);
    }

    public Pose2d getPose() {
        
        // System.out.println("getPose");
        return this.m_odometry.getEstimatedPosition();
    
    }    

    // private SwerveModuleState[] getModuleStates() {
    //     return this.m_moduleStates;
    // }

    private ChassisSpeeds getSpeeds() {
        return Constants.DrivetrainConstants._kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public InstantCommand resetGyro() {
        return new InstantCommand(() -> m_pigeon2.setYaw(0));
    }

    private String getCurrentState() {
        return _currentState.name();
    }
    
    private String getPreviousState() {
        return _previousState.name();
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
        return this.m_pigeon2.getAngle();
    }
    //#endregion

    private void updateOdometry() {
        // _vision.imposeVisionMeasurements(this);
        ArrayList<VisionMeasurement> visionMeasurements = _vision
            .getVisionMeasurements(

            m_pigeon2.getRotation2d()
        );

        for (VisionMeasurement visionMeasurement : visionMeasurements) {
            addVisionMeasurement(
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
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
        this.periodicIO.VxCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftY());

        // The Y translation will be the horizontal value of the left driver joystick
        this.periodicIO.VyCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.XY_Axis_inputBreakpoints, Constants.DrivetrainConstants.XY_Axis_outputTable, _driverController.getLeftX());

        // The rotation will be the horizontal value of the right driver joystick
        this.periodicIO.WzCmd = -OneDimensionalLookup.interpLinear(Constants.DrivetrainConstants.RotAxis_inputBreakpoints, Constants.DrivetrainConstants.RotAxis_outputTable, _driverController.getRightX());
        targetRotation = GeometryUtil.getAngleToTarget(_target, this::getPose, _isFollowingFront) / 50;

        updateOdometry();
        handleCurrentState().schedule();
    }

    private Command handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                return applyRequest(() -> idle);
            case OPEN_LOOP:
                return applyRequest(() -> drive.withVelocityX(this.periodicIO.VxCmd * DrivetrainConstants.kSpeedAt12VoltsMps)
            .withVelocityY(this.periodicIO.VyCmd * DrivetrainConstants.kSpeedAt12VoltsMps)
            .withRotationalRate(this.periodicIO.WzCmd * DrivetrainConstants.MaxAngularRate));
            case TRAJECTORY_FOLLOW:
                //return applyRequest(() -> chassisDrive.withSpeeds(followChassisSpeeds));
            //     return applyRequest(() -> drive.withVelocityX(this.followChassisSpeeds.vxMetersPerSecond * DrivetrainConstants.kSpeedAt12VoltsMps)
            // .withVelocityY(this.followChassisSpeeds.vyMetersPerSecond * DrivestrainConstants.kSpeedAt12VoltsMps)
            // .withRotationalRate(this.followChassisSpeeds.omegaRadiansPerSecond * DrivetrainConstants.MaxAngularRate));
            return new InstantCommand();
            case TARGET_FOLLOW:
                return applyRequest(() -> drive.withVelocityX(this.periodicIO.VxCmd * DrivetrainConstants.kSpeedAt12VoltsMps)
                .withVelocityY(this.periodicIO.VyCmd * DrivetrainConstants.kSpeedAt12VoltsMps)
                .withRotationalRate(targetRotation * DrivetrainConstants.MaxAngularRate));
            case NOTE_FOLLOW:
                return handleNoteFollow();
            // case MOVE_TO_NOTE:
            //     handleMoveToNote();
            //     break;
            default:
                return applyRequest(() -> idle);

        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {  
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private Command handleNoteFollow()
    {

        var rotation = _vision.getNoteAngle(); 
        return applyRequest(() -> drive.withVelocityX(this.periodicIO.VxCmd * DrivetrainConstants.kSpeedAt12VoltsMps)
                .withVelocityY(this.periodicIO.VyCmd * DrivetrainConstants.kSpeedAt12VoltsMps)
                .withRotationalRate(-rotation * DrivetrainConstants.MaxAngularRate * 0.012));
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
