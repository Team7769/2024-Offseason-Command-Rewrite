package frc.robot.enums;

import frc.robot.statemachine.IState;

public enum JukeboxState implements IState {
        /**Idle state, most of the match. */
        IDLE,

        /**Score the note. */
        SCORE,

        /**Spins up the shooter for a speaker shot. */
        PREP_SPEAKER,

        /**Spins up the shooter for a speaker shot from the podium (Doesn't use auto aim) */
        PREP_SPEAKER_PODIUM,

        /**Spins up the shooter for a speaker shot from the white line (Doesn't use auto aim) */
        PREP_SPEAKER_LINE,

        /**Spins up the shooter for a speaker shot from the subwoofer (Doesn't use auto aim and only used for auto) */
        PREP_SPEAKER_SUBWOOFER,

        /**Spins up the shooter to launch across the field  */
        PREP_LAUNCH,

        /**Angles the jukebox to intake from the human element directly */
        PREP_HUMAN_INTAKE,

        /**Brings the jukebox to the amp scoring position. */
        PREP_AMP,

        /**Brings the jukebox to the trap scoring position. */
        PREP_TRAP,

        /**Resets the jukebox to the IDLE configuration. */
        RESET,

        /**Sets the elevator up for emergency clearing of notes */
        EJECT,

        /**Extends the elevator for climbing in the endgame. */
        EXTEND_FOR_CLIMB,

        /**Retracts the elevator to climb in the endgame. */
        CLIMB,

        /**Enables change in jukebox variables for shots from the dashboard */
        JUKEBOX_TEST,
        
        /**Manual control */
        MANUAL
}