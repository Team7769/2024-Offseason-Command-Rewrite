package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.enums.JukeboxState;
import frc.robot.statemachine.StateBasedSubsystem;
import frc.robot.utilities.OneDimensionalLookup;

public class Jukebox extends StateBasedSubsystem<JukeboxState> {

    // Motor Controllers
    private CANSparkMax _elevatorL;
    private CANSparkMax _elevatorR;
    private CANSparkMax _feeder;
    private CANSparkMax _shooterAngle;
    private CANSparkMax _shooterL;
    private CANSparkMax _shooterR;

    private double _targetDistance = 0;

    // Motor Controller PIDs
    private SparkPIDController _shooterAngleController;
    private SparkPIDController _elevatorController;
    private SparkPIDController _shooterController;
    private SparkPIDController _shooterRController;

    // Jukebox State Control
    private JukeboxState jukeboxCurrentState = JukeboxState.IDLE;
    private JukeboxState jukeboxPreviousState = JukeboxState.IDLE;

    // Elevator Profile
    private ElevatorFeedforward _elevatorFeedForward;
    private TrapezoidProfile _elevatorProfile;
    private TrapezoidProfile.State _elevatorProfileGoal;
    private TrapezoidProfile.State _elevatorProfileSetpoint;
    
    // Shooter Angle Profile
    private ArmFeedforward _shooterAngleFeedForward;
    private TrapezoidProfile _shooterAngleProfile;
    private TrapezoidProfile.State _shooterAngleProfileGoal;
    private TrapezoidProfile.State _shooterAngleProfileSetpoint;
    
    // Shooter Profile
    private SimpleMotorFeedforward _shooterFeedForward;
    private double _shooterSetpoint;

    private DigitalInput _noteHolderPE;
    private DigitalInput _noteShooterPE;
    private Debouncer _noteHolderPEDebouncer;
    private Debouncer _noteShooterPEDebouncer;
    private Debouncer _shootReadyDebouncer;
    private Boolean _inNoteHolder = false;
    private Boolean _inNoteShooter = false;
    private Boolean _isReadyToShoot = false;

    private double _manualElevatorSpeed = 0;
    private double _manualFeederSpeed = 0;
    private double _manualShooterAngleSpeed = 0;
    private double _manualShooterSpeed = 0;
    private double kShooterIdleSpeed = 38;

    private Vision _vision;

    private double _dashboardShooterTargetSpeed = 0.0;
    private double _dashboardShooterTargetAngle = 0.0;
    private double _dashboardShooterRPercent =  0.9;
    private double _shooterSetpointRpm = 0.0;

    private int _loopCounter = 0;
    
    public Jukebox() {

    }

    /**
     * Handles the position of the elevator using a Trapezoidal Motion Profile
     * The value should be set using setElevatorPosition. 
     * This method should be called during handleCurrentState()
     */
    private void handleElevatorPosition() {
        // This is the correct logic per the example at https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/elevatortrapezoidprofile/Robot.java
        // t = time since last update, _elevatorProfileSetpoint is last state, _goal is the target state
        _elevatorProfileSetpoint = _elevatorProfile.calculate(
            Constants.JukeboxConstants.kTInterval,
            _elevatorProfileSetpoint,
            _elevatorProfileGoal
        );

        // Check that this value is high enough for it to move.
        double calculatedFeedForward = _elevatorFeedForward.calculate(
            _elevatorProfileSetpoint.velocity
        );

        // Test this as is. If this doesn't move, then multiply ff by 12 to get Volts. SetReference is expecting a voltage for the FF value here.
        // Just a note, I think our ElevatorFeedfowardkV value is a little bit low. The elevator for last year was .066 and this one is .001 This could be the cause. We can tune this if needed.
        if (_elevatorL.getEncoder().getPosition() >= 84 && _elevatorProfileSetpoint.velocity > 0) {
           _elevatorL.set(0);
        } else { 
            _elevatorController.setReference(_elevatorProfileSetpoint.position,
                            CANSparkBase.ControlType.kPosition,
                            0,
                            calculatedFeedForward);
        }

        // This is being printed to Shuffleboard
        // Check that these position/velocity values are changing toward the setpoint, 5 in our test case.
        // Should be going closer to the target
        SmartDashboard.putNumber("profileSetpointPosition", _elevatorProfileSetpoint.position);

        // Should be going up then slowing down closer to the target
        SmartDashboard.putNumber("profileSetpointVelocity", _elevatorProfileSetpoint.velocity);
        
        // You can display these 3 values as a graph to see how they are proceeding.
        SmartDashboard.putNumber("elevatorFeedforward", calculatedFeedForward);
    }
    
    /**
     * Handles the position of the shooter angle using a Trapezoidal Motion Profile
     * The value should be set using setShooterAngle. 
     * This method should be called during handleCurrentState()
     */
    private void handleShooterAnglePosition() {
        // t = time since last update, _shooterAngleProfileSetpoint is last state, _goal is the target state
        _shooterAngleProfileSetpoint = _shooterAngleProfile.calculate(
            Constants.JukeboxConstants.kTInterval,
            _shooterAngleProfileSetpoint,
            _shooterAngleProfileGoal
        );

        double calculatedFeedForward = _shooterAngleFeedForward.calculate(
            Constants.JukeboxConstants.kShooterAngleFeedForwardAngle +
            _shooterAngleProfileSetpoint.position,
            _shooterAngleProfileSetpoint.velocity
        );

        _shooterAngleController.setReference(
            _shooterAngleProfileSetpoint.position,
            CANSparkBase.ControlType.kPosition,
            0,
            calculatedFeedForward);

        SmartDashboard.putNumber("angleProfileSetpointPosition",
                                 _shooterAngleProfileSetpoint.position);

        SmartDashboard.putNumber("angleProfileSetpointVelocity",
                                 _shooterAngleProfileSetpoint.velocity);

        SmartDashboard.putNumber("shooterAngleFeedforward",
                                 calculatedFeedForward);

    }
    
    /**
     * Handles the velocity of the Shooter using a Trapezoidal Motion Profile
     * The value should be set using setShooterSpeed. 
     * This method should be called during handleCurrentState()
     */
    private void handleShooterSpeed() {
        double calculatedFeedForward = _shooterFeedForward.calculate(
            _shooterSetpoint
        );
        double calculatedFeedForwardR = _shooterFeedForward.calculate(_shooterSetpoint * _dashboardShooterRPercent);

        _shooterSetpointRpm = _shooterSetpoint * 60;

        _shooterController.setReference(_shooterSetpointRpm,
                                        CANSparkBase.ControlType.kVelocity,
                                        0,
                                        calculatedFeedForward);
        _shooterRController.setReference(_shooterSetpointRpm * _dashboardShooterRPercent,
                                        CANSparkBase.ControlType.kVelocity,
                                        0,
                                        calculatedFeedForwardR);
        SmartDashboard.putNumber("shooterSetpointRpm", _shooterSetpointRpm);
        SmartDashboard.putNumber("shooterFeedforward", calculatedFeedForward);
    }

    /**
     * Method that will set the angle of the shooter.
     * This should also be apply to the tilt angle too.
     * Once the shooter angle is set it should auto apply the tilt angle.
     * 1 means it set it clockwise
     * -1 means it set it counterclockwise
     */
    private void setShooterAngle(double desiredPosition) {
        // _shooterAngleController.setReference(desiredPosition, com.revrobotics.CANSparkBase.ControlType.kPosition, 0);
        _shooterAngleProfileGoal = new TrapezoidProfile.State(desiredPosition,
                                                              0);
    }

    /**
     * Sets the elevator to where it needs to be and if the position changes we reset the timer and update the old position to the new position
     * @param position takes a double and makes the goal state.
     */
    private void setElevatorPosition(double desiredPosition)
    {
        // Set the profile goal
        _elevatorProfileGoal = new TrapezoidProfile.State(desiredPosition, 0);
    }

    /**
     * ShooterDeploy ramps up the shooter motors to the max
     */
    private void setShooterSpeed(double v)
    {
        _shooterSetpoint = v;
    }

    private void score() {
        // If previous state is PREP_AMP or PREP_TRAP -> Reverses out the front of the robot.
        // If previous state is PREP_SPEAKER -> Forward into the shooter motors in the back.
        if (jukeboxPreviousState == JukeboxState.PREP_AMP) {

            _feeder.set(-Constants.JukeboxConstants.kFeederShootSpeed);
        } else if (jukeboxPreviousState == JukeboxState.PREP_TRAP)
        {            
            if (hasNote()) {
                _feeder.set(-.1);
            } else {
                _feeder.set(-.075); 
            }
            // if (!hasNote()) {
            //     if (_loopCounter < 10) {
            //         _feeder.set(-0.1);
            //     } else {
            //         _feeder.set(0);
            //     }
            // } else {
            //     _feeder.set(-0.1);
            // }
        } else if (jukeboxPreviousState == JukeboxState.PREP_SPEAKER || 
                    jukeboxPreviousState == JukeboxState.PREP_SPEAKER_PODIUM ||
                    jukeboxPreviousState == JukeboxState.PREP_SPEAKER_LINE ||
                    jukeboxPreviousState == JukeboxState.PREP_LAUNCH ||
                    jukeboxPreviousState == JukeboxState.JUKEBOX_TEST ||
                    jukeboxPreviousState == JukeboxState.PREP_SPEAKER_SUBWOOFER) {
            _feeder.set(Constants.JukeboxConstants.kFeederShootSpeed);
        }
        if (_loopCounter >= 50) {
            _loopCounter = 0;
        }
        _loopCounter++;
    }

    private void prepAmp() {
        feeder();
        setShooterSpeed(0);
        setShooterAngle(Constants.JukeboxConstants.kFeedShooterAngle);
        setElevatorPosition(Constants.JukeboxConstants.kAmpElevatorPosition); 
    }

    private void prepTrap() {
        if (jukeboxPreviousState != JukeboxState.CLIMB && jukeboxPreviousState != JukeboxState.SCORE) return;
        if (jukeboxPreviousState == JukeboxState.SCORE) {
            _feeder.set(.1);
        } else {
            _feeder.set(0);
        }
        setShooterSpeed(0.0);

            setShooterAngle(Constants.JukeboxConstants.kTrapShooterAngle);
                //setElevatorPosition(83);
                
                setElevatorPosition(Constants.JukeboxConstants.kTrapElevatorPosition);

        // var elevatorPosition = _elevatorL.getEncoder().getPosition();
        // if (elevatorPosition < 3) {
        //     setElevatorPosition(5);
        // } else if (elevatorPosition >= 3) {
        //     if (_shooterAngle.getEncoder().getPosition() >= 7)
        //     {
        //     }
        // }
    }

    private void prepSpeaker() {
        setElevatorPosition(0);

        _targetDistance = _vision.getDistance();

        feeder();

        if (_targetDistance != 0.0) {
            double desiredShooterAngle = OneDimensionalLookup.interpLinear(
                Constants.JukeboxConstants.kDistanceIDs,
                Constants.JukeboxConstants.kShooterAngles,
                _targetDistance
            );
            double desiredShooterSpeed = OneDimensionalLookup.interpLinear(
                Constants.JukeboxConstants.kDistanceIDs,
                Constants.JukeboxConstants.kShooterSpeeds,
                _targetDistance
            );
            setShooterAngle(desiredShooterAngle);
            setShooterSpeed(desiredShooterSpeed);

        } else {
            setShooterAngle(Constants.JukeboxConstants.KMinShooterAngle);
            setShooterSpeed(Constants.JukeboxConstants.kMaxShooterSpeed);
        }
    }

    /** Preps the speaker for a shot from the podium (Doesn't use auto aim) */
    private void prepSpeakerPodium() {
        feeder();
        setShooterAngle(Constants.JukeboxConstants.kPodiumSpeakerShotAngle);
        setShooterSpeed(Constants.JukeboxConstants.kPodiumSpeakerShotSpeed);
        setElevatorPosition(0);
    }

    private void prepSpeakerLine() {
        feeder();
        setShooterAngle(Constants.JukeboxConstants.kLineSpeakerShotAngle);
        setShooterSpeed(Constants.JukeboxConstants.kLineSpeakerShotSpeed);
        setElevatorPosition(0);
    }

    private void prepSpeakerSubwoofer() {
        feeder();
        setShooterAngle(Constants.JukeboxConstants.KMinShooterAngle);
        setShooterSpeed(Constants.JukeboxConstants.kMaxShooterSpeed);
        setElevatorPosition(0);
    }

    private void prepLaunch() {
        feeder();
        setShooterAngle(Constants.JukeboxConstants.kLaunchAngle);
        setShooterSpeed(Constants.JukeboxConstants.kLaunchSpeed);
        setElevatorPosition(0);
    }

    private void prepHumanIntake() {
        setShooterAngle(Constants.JukeboxConstants.kHumanElementIntakeAngle);
        setElevatorPosition(0);
    }

    private void reset() {
        setShooterAngle(0.0);
        setShooterSpeed(0.0);
    }

    private void extendForClimb() {
        setShooterSpeed(0);
        if (_shooterL.getEncoder().getVelocity() < 5) {
            _feeder.set(Constants.JukeboxConstants.kFeederIntake);
        }
        setShooterAngle(Constants.JukeboxConstants.kExtendClimbShooterAngle);
        setElevatorPosition(Constants.JukeboxConstants.kExtendClimbElevatorPosition);
        // _elevatorProfileSetpoint = new TrapezoidProfile.State(90, 0);
        // if (_elevatorL.getEncoder().getPosition() < 83) {
        //     _elevatorL.set(.6);
        // } else {
        //     _elevatorL.set(.02);
        // }
    }

    private void climb() {
        _feeder.set(0);
        if (jukeboxPreviousState != JukeboxState.EXTEND_FOR_CLIMB) {
            return;
        }
        setElevatorPosition(0);
        _elevatorProfileSetpoint = new TrapezoidProfile.State();
        if (_elevatorL.getEncoder().getPosition() >= 0) {
            _elevatorL.set(-.5);
        } else {
            _elevatorL.set(0);
        }
    }

    private void emergencyEject() {
        // setElevatorPosition(kEmergencyEjectElevatorPosition);
        setShooterAngle(Constants.JukeboxConstants.kEmergencyEjectShooterAngle);
    }

    private void feeder()
    {
        if (_inNoteShooter) {
            _feeder.set(Constants.JukeboxConstants.kFeederReverse); 
        } else if (_inNoteHolder) {
            _feeder.set(0);
        } else {
            _feeder.set(Constants.JukeboxConstants.kFeederIntake);
        }
    }
 
    public void setManualFeederSpeed(double givenSpeed)
    {
        _manualFeederSpeed = givenSpeed;
    }

    private void idle() {
        setElevatorPosition(0.5); // .5 so the elevator doesn't keep trying to reach zero even when at the bottom
        setShooterAngle(0);
        if (!Constants.JukeboxConstants._disableAutoSpinup) {
            setShooterSpeed(kShooterIdleSpeed);
        } else {
            setShooterSpeed(0);
        }

        feeder();
    }

    private void manual() {
        _elevatorL.set(_manualElevatorSpeed);
        _shooterL.set(_manualShooterSpeed);
        _shooterAngle.set(_manualShooterAngleSpeed);
        _feeder.set(_manualFeederSpeed);
    }

    private void jukeboxTest() {
        feeder();
        setShooterAngle(_dashboardShooterTargetAngle);
        setShooterSpeed(_dashboardShooterTargetSpeed);
    }

    public void setManualElevatorSpeed(double givenSpeed)
    {
        _manualElevatorSpeed = givenSpeed;
    }

    public void setManualShooterSpeed(double speed) {
        _manualShooterSpeed = speed;
    }

    public void setManualShooterAngleSpeed(double speed) {
        _manualShooterAngleSpeed = speed;
    }

    public void resetSensors()  {
        _elevatorL.getEncoder().setPosition(0.0);
        _shooterAngle.getEncoder().setPosition(0.0);
    }

    public boolean hasNote() {
        // get() returns false if blocked/detects note
        return _inNoteHolder || _inNoteShooter;
    }

    public double getElevatorPosition() {
        return _elevatorL.getEncoder().getPosition();
    }

    public void setIdleSpeedSubwoofer() {
        kShooterIdleSpeed = 38;
    }

    public void setIdleSpeedMax() {
        kShooterIdleSpeed = 67;
    }

    public boolean getDisableAutoSpinup() {
        return Constants.JukeboxConstants. _disableAutoSpinup;
    }

    public void disableAutoSpinup() {
        Constants.JukeboxConstants._disableAutoSpinup = true;
    }

    public void enableAutoSpinup() {
        Constants.JukeboxConstants._disableAutoSpinup = false;
    }

    public int getShooterLeds(int numLeds) {
        var target = _shooterSetpointRpm + 300;
        if (target > 0) {
            double shooterProportion = numLeds * (_shooterL.getEncoder().getVelocity() / (target));
            return (int)Math.ceil(shooterProportion);
        } else {
            return 0;
        }

    }

    public void handleCurrentState()
    {
        switch(jukeboxCurrentState) {
            case MANUAL:
                manual();
                break;
            case JUKEBOX_TEST:
                jukeboxTest();
                break;
            case SCORE:
                score();
                break;
            case PREP_SPEAKER:
                prepSpeaker();
                break;
            case PREP_SPEAKER_PODIUM:
                prepSpeakerPodium();
                break;
            case PREP_SPEAKER_LINE:
                prepSpeakerLine();
                break;
            case PREP_SPEAKER_SUBWOOFER:
                prepSpeakerSubwoofer();
                break;
            case PREP_LAUNCH:
                prepLaunch();
                break;
            case PREP_HUMAN_INTAKE:
                prepHumanIntake();
            case PREP_AMP:
                prepAmp();
                break;
            case PREP_TRAP:
                prepTrap();
                break;
            case RESET:
                reset();
            case EJECT:
                emergencyEject();
                break;
            case EXTEND_FOR_CLIMB:
                extendForClimb();
                break;
            case CLIMB:
                climb();
                break;
            default:
                idle();
                break;
        }

        switch (jukeboxCurrentState) {
            case MANUAL:
            case CLIMB:
                break;
            // case EXTEND_FOR_CLIMB:
            //     handleShooterSpeed();
            //     handleShooterAnglePosition();
            //     break;
            default:
                handleShooterSpeed();
                handleShooterAnglePosition();
                handleElevatorPosition();
                break;
        }
    }

    @Override
    public void periodic()
    {
        handleCurrentState();
    }
}
