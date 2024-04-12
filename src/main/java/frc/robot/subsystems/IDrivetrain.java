package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.enums.DrivetrainState;

public interface IDrivetrain {
    /**
     * Sets the wanted Drivetrain State. Does nothing if the Drivetrain is already in the wanted state.
     * @param state The state to transition to.
     * @return The instant command.
     */
    public InstantCommand setWantedState(DrivetrainState state);

    /**
     * Targets the speaker.
     * @return The sequential command group to execute the action.
     */
    public SequentialCommandGroup targetSpeaker(Supplier<Boolean> isRedAlliance);
    
    /**
     * Targets the zone.
     * @return The sequential command group to execute the action.
     */
    public SequentialCommandGroup targetZone(Supplier<Boolean> isRedAlliance);
}
