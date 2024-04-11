package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.enums.DrivetrainState;
import frc.robot.subsystems.IDrivetrain;

public class TeleopInitializeCommand extends InstantCommand {
    private final IDrivetrain drivetrain;

    public TeleopInitializeCommand(final IDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize()
    {
        this.drivetrain.setWantedState(DrivetrainState.OPEN_LOOP);
    }
}
