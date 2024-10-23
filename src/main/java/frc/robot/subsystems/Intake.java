package frc.robot.subsystems;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.enums.IntakeState;
import frc.robot.statemachine.StateBasedSubsystem;
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

        _currentState = IntakeState.STOP;
        _previousState = IntakeState.PASSIVE_EJECT;

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

    @Override
    public void periodic()
    {
        handleCurrentState();
    }
}