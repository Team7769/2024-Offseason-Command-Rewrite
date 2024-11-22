package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.enums.DrivetrainState;
import frc.robot.enums.IntakeState;
import frc.robot.statemachine.StateBasedSubsystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

public class Intake extends StateBasedSubsystem<IntakeState>
{
    private CANSparkMax _motor;


    public Intake() {
        _motor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorId,
                                 MotorType.kBrushless);

        _motor.setIdleMode(IdleMode.kBrake);
        _motor.setSmartCurrentLimit(Constants.IntakeConstants.kMotorStallLimit, Constants.IntakeConstants.kMotorFreeLimit);
        _motor.setInverted(Constants.IntakeConstants.kInverted);
        _motor.burnFlash();

        _currentState = IntakeState.PASSIVE_EJECT;
        _previousState = IntakeState.STOP;
        //_jukebox = Jukebox.getInstance();
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        var stateLayout = tab.getLayout("State", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
        stateLayout.addString("Intake: Current State", this::getCurrentStateName);
        stateLayout.addString("Intake: Previous State", this::getPreviousStateName);
    }

    public Command stop() {
        return run(() -> _motor.set(Constants.IntakeConstants.kStopSpeed));
    }
    
    public Command intake() {
        return run(() -> _motor.set(Constants.IntakeConstants.kIntakeSpeed));
    }

    // emergency eject
    public Command eject() {
        return run(() -> _motor.set(Constants.IntakeConstants.kEjectSpeed));
    }

    // when we have a note, slowly turn the motor in reverse to avoid sucking
    // notes in
    public Command passiveEject() {
        return run(() -> _motor.set(Constants.IntakeConstants.kPassiveEjectSpeed));
    }

    public Command handleCurrentState() {
        switch (_currentState) {
            case STOP:
                return stop();
            
            case INTAKE:
                return intake();

            case PASSIVE_EJECT:
                // when we have a note, slowly turn the motor in reverse to 
                // avoid sucking notes in
                return passiveEject();

            case EJECT:
                // emergency eject
                return eject();
            default:
            return stop();
        }
    }

    @Override
    public InstantCommand setWantedState(IntakeState state) {
        return new InstantCommand(() -> {
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;
            }
        }, this);
    }

    @Override
    public void periodic()
    {
        handleCurrentState().schedule();
        
    }
}