package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
    private Pose2d _limelightPose = new Pose2d();

    private PhotonPoseEstimator[] _photonPoseEstimators;

    private final String _targeterLimelightName;
    private final String[] _limelightNames;
    private PhotonCamera[] _photonCameras;

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

            _photonPoseEstimators[i] = new PhotonPoseEstimator(
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                photonCamera,
                Constants.VisionConstants.kRobotCamOffsets[i]
            );

            _photonCameras[i] = photonCamera;
        }
    }

    public void imposeVisionMeasurements(SwerveDrivePoseEstimator poseEstimator) {
        for (int i = 0; i < _photonCameras.length; i++) {
            // PhotonCamera photonCamera = _photonCameras[i];

            PhotonPoseEstimator photonPoseEstimator =
                _photonPoseEstimators[i];

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

        Pose2d masterPose = poseEstimator.getEstimatedPosition();

        for (String limelightName : _limelightNames) {
            LimelightHelpers.SetRobotOrientation(
                limelightName,
                masterPose.getRotation().getDegrees(),
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
}
