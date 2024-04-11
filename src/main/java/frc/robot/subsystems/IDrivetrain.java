package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.enums.DrivetrainState;

public interface IDrivetrain extends Subsystem {
    public void setWantedState(DrivetrainState state);
    public void setTarget(Translation2d targetPosition, boolean isFollowingFront);
    public void setTrajectoryFollowModuleTargets(ChassisSpeeds robotRelativeSpeeds);
}
