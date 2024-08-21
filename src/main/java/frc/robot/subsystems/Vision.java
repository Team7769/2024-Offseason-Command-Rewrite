package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
    private Pose2d _limelightPose = new Pose2d();

    private ArrayList<PhotonPoseEstimator> _photonPoseEstimators = new ArrayList<>();
    private ArrayList<PhotonCamera> _photonCameras = new ArrayList<>();

    private final String _targeterLimelightName;
    private final String[] _limelightNames;

    public Vision(
        String targeterLimelightName, 
        String[] limelightNames, 
        String[] photonCameraNames
    ) {
        _targeterLimelightName = targeterLimelightName;
        _limelightNames = limelightNames;

        for (int i = 0; i < photonCameraNames.length; i++) {
            String photonCameraName = photonCameraNames[i];

            PhotonCamera photonCamera = new PhotonCamera(photonCameraName);

            _photonPoseEstimators.add(new PhotonPoseEstimator(
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                photonCamera,
                Constants.VisionConstants.kPhotonCameraOffsets[i]
            ));

            _photonCameras.add(photonCamera);
        }
    }

    public void updateLimelightPosition(Rotation2d rotation) {
        for (String limelightName : _limelightNames) {
            LimelightHelpers.SetRobotOrientation(
                limelightName,
                rotation.getDegrees(),
                0,
                0,
                0,
                0,
                0
            );
        }
    }

    public void imposeVisionMeasurements(
        SwerveDrivePoseEstimator poseEstimator, Rotation2d rotation
    ) {
        for (int i = 0; i < _photonCameras.size(); i++) {
            // PhotonCamera photonCamera = _photonCameras[i];

            PhotonPoseEstimator photonPoseEstimator =
                _photonPoseEstimators.get(i);

            EstimatedRobotPose photonPoseEstimate =
                photonPoseEstimator.update().get();

            // iif (estimatedRobotPose == null) return;

            // PhotonPipelineResult camResult = photonCamera.getLatestResult();

            // poseEstimator.setVisionMeasurementStdDevs(null);

            poseEstimator.addVisionMeasurement(
                photonPoseEstimate.estimatedPose.toPose2d(),
                photonPoseEstimate.timestampSeconds
            );
        }

        for (String limelightName : _limelightNames) {
            LimelightHelpers.SetRobotOrientation(
                limelightName,
                rotation.getDegrees(),
                0, 
                0, 
                0, 
                0,
                0
            );

            PoseEstimate limelightPoseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                    limelightName
                );

            poseEstimator.addVisionMeasurement(
                limelightPoseEstimate.pose,
                limelightPoseEstimate.timestampSeconds
            );
        }
    }

    @Override
    public void periodic() {
        _limelightPose = LimelightHelpers.getBotPose2d("limelight");
    }

    public Pose2d getLimelightPose() {
        return _limelightPose;
    }

    public double getNoteAngle()
    {
        double tx = LimelightHelpers.getTX("limelight");
        SmartDashboard.putNumber("NoteAngle", tx);
        return tx;
    }

    public double getNoteDistance()
    {
        double ty = LimelightHelpers.getTY("limelight");
        SmartDashboard.putNumber("NoteAngle", ty);
        return ty;
    }


}
