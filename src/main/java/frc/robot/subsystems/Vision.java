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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.VisionMeasurement;

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

        // for (int i = 0; i < photonCameraNames.length; i++) {
        //     String photonCameraName = photonCameraNames[i];

        //     PhotonCamera photonCamera = new PhotonCamera(photonCameraName);

        //     _photonPoseEstimators.add(new PhotonPoseEstimator(
        //         AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
        //         PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        //         photonCamera,
        //         Constants.VisionConstants.kPhotonCameraOffsets[i]
        //     ));

        //     _photonCameras.add(photonCamera);
        // }
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

    public ArrayList<VisionMeasurement> getVisionMeasurements(
        Rotation2d rotation
    ) {
        for (
            PhotonPoseEstimator photonPoseEstimator : _photonPoseEstimators
        ) {
            // PhotonCamera photonCamera = _photonCameras[i];

            // PhotonPoseEstimator photonPoseEstimator =
            //     _photonPoseEstimators.get(i);

            // EstimatedRobotPose photonPoseEstimate =
            //     photonPoseEstimator.update();

            // iif (estimatedRobotPose == null) return;

            // PhotonPipelineResult camResult = photonCamera.getLatestResult();

            // poseEstimator.setVisionMeasurementStdDevs(null);

            // poseEstimator.addVisionMeasurement(
            //     photonPoseEstimate.estimatedPose.toPose2d(),
            //     photonPoseEstimate.timestampSeconds
            // );
        }

        ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<>();

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

            if (limelightPoseEstimate.tagCount > 0) {
                visionMeasurements.add(
                    new VisionMeasurement(
                        limelightPoseEstimate.pose,
                        limelightPoseEstimate.timestampSeconds
                    )
                );
            }

            // poseEstimator.addVisionMeasurement(
            //     limelightPoseEstimate.pose,
            //     limelightPoseEstimate.timestampSeconds
            // );
        }

        return visionMeasurements;
    }

    @Override
    public void periodic() {
        _limelightPose = LimelightHelpers.getBotPose2d("limelight");
    }

    public Pose2d getLimelightPose() {
        return _limelightPose;
    }
}
