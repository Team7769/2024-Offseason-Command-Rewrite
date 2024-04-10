package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain.DrivetrainState;

public class TargetSpeakerCommand extends InstantCommand {
    private final Drivetrain drivetrain;

    public TargetSpeakerCommand(final Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize()
    {
        this.drivetrain.setTarget(Constants.FieldConstants.kSpeaker, false);
        this.drivetrain.setWantedState(DrivetrainState.TARGET_FOLLOW);
    }
}
