package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.enums.IntakeState;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase
{
    private CANSparkMax _motor;

    private IntakeState _currentState = IntakeState.STOP;
    private IntakeState _previousState = IntakeState.PASSIVE_EJECT;

    public Intake() {
        _motor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorId,
                                 MotorType.kBrushless);

        _motor.setIdleMode(IdleMode.kBrake);
        _motor.setSmartCurrentLimit(Constants.IntakeConstants.kMotorStallLimit, Constants.IntakeConstants.kMotorFreeLimit);
        _motor.setInverted(Constants.IntakeConstants.kInverted);
        _motor.burnFlash();

        //_jukebox = Jukebox.getInstance();
    }

    public void stop() {
        _motor.set(Constants.IntakeConstants.kStopSpeed);
    }
    
    public void intake() {
        // if we have a note, change to passive eject mode

        // if (_jukebox.hasNote()) {
        //     setWantedState(IntakeState.PASSIVE_EJECT);

        //     return;
        // }

        _motor.set(Constants.IntakeConstants.kIntakeSpeed);
    }

    // emergency eject
    public void eject() {
        _motor.set(Constants.IntakeConstants.kEjectSpeed);
    }

    // when we have a note, slowly turn the motor in reverse to avoid sucking
    // notes in
    public void passiveEject() {
        // if we dont have a note, change to intake mode

        // if (!_jukebox.hasNote()) {
        //     
        //     setWantedState(IntakeState.INTAKE);

        //     return;
        // }

        _motor.set(Constants.IntakeConstants.kPassiveEjectSpeed);
    }

    public void handleCurrentState() {
        switch (_currentState) {
            case STOP:
                stop();

                break;
            
            case INTAKE:
                intake();

                break;

            case PASSIVE_EJECT:
                // when we have a note, slowly turn the motor in reverse to 
                // avoid sucking notes in
                passiveEject();

                break;

            case EJECT:
                // emergency eject
                eject();

                break;

            default:
                break;
        }
    }

    public InstantCommand setWantedState(IntakeState state) {
        return new InstantCommand(() -> {
            System.out.print(state.name());
            if (state != _currentState) {
                _previousState = _currentState;
                _currentState = state;
            }
        }, this);
    }

    @Override
    public void periodic()
    {
        handleCurrentState();
    }
    
    
}