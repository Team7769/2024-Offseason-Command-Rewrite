package frc.robot.enums;

import frc.robot.statemachine.IState;

public enum IntakeState implements IState {
    // use preferably only when robots disabled
    STOP,

    // run when we don't have a note
    INTAKE,

    // run in reverse (of intake) when we have a note to avoid intake
    PASSIVE_EJECT,

    // eject anything we have at full speed, for emergencies
    EJECT
}