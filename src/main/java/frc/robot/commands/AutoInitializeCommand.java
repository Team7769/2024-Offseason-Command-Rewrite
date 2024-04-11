package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.enums.DrivetrainState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IDrivetrain;

public class AutoInitializeCommand extends InstantCommand {
    private final IDrivetrain drivetrain;

    public AutoInitializeCommand(final IDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize()
    {
        this.drivetrain.setTrajectoryFollowModuleTargets(new ChassisSpeeds());
        this.drivetrain.setWantedState(DrivetrainState.TRAJECTORY_FOLLOW);
    }
}
