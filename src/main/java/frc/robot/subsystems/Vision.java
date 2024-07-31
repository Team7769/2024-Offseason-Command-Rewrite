package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    private Pose2d _limelightPose = new Pose2d();

    private final String _limelightName;

    public Vision(String limelightName) {
        _limelightName = limelightName;
    }

    public Vision() {
        this("limelight");
    }

    @Override
    public void periodic() {
        _limelightPose = LimelightHelpers.getBotPose2d("limelight");
    }

    public Pose2d getLimelightPose() {
        return _limelightPose;
    }
}
