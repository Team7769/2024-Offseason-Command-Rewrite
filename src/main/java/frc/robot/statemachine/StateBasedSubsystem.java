package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateBasedSubsystem<T extends IState> extends SubsystemBase {
    protected T _currentState;
    protected T _previousState;
    
    public T getCurrentState() {
        return _currentState;
    }

    public T getPreviousState() {
        return _previousState;
    }

    public InstantCommand setWantedState(T state){
        return new InstantCommand(() -> {
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;
            }
        });
    }
}