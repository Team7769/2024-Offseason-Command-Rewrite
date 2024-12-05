package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.enums.JukeboxState;
import frc.robot.enums.LocationTarget;
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

    // private Vision _vision;

    private SDSDrivetrain _drivetrain;

    private double _dashboardShooterTargetSpeed = 0.0;
    private double _dashboardShooterTargetAngle = 0.0;
    private double _dashboardShooterRPercent =  0.9;
    private double _shooterSetpointRpm = 0.0;

    private int _loopCounter = 0;

    private LocationTarget target = LocationTarget.NONE;
    


    public Jukebox(SDSDrivetrain drivetrain)
    {
        _drivetrain = drivetrain;
        // Config Elevator
        configElevator();

        // Config Shooter Angle
        configShooterAngle();

        // Config Shooter
        configShooter();

        // Config Feeder
        configFeeder();

        //_visionSystem = VisionSystem.getInstance();

        // Jukebox State Control
        _currentState = JukeboxState.IDLE;
        _previousState = JukeboxState.IDLE;

        ShuffleboardTab tab = Shuffleboard.getTab("Jukebox");
        var stateLayout = tab.getLayout("State", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
        stateLayout.addString("Jukebox: Current State", this::getCurrentStateName);
        stateLayout.addString("Jukebox: Previous State", this::getPreviousStateName);
    }

    /**
     * Configures the Feeder Controllers and Sensors
     */
    private void configFeeder() {
        // feeder motor setup
        _feeder = new CANSparkMax(Constants.JukeboxConstants.kFeederId, MotorType.kBrushless);
        _feeder.setIdleMode(IdleMode.kBrake);
        _feeder.setInverted(true);
        _feeder.burnFlash();

        _noteHolderPE = new DigitalInput(Constants.JukeboxConstants.kNoteHolderPEChannel);
        _noteShooterPE = new DigitalInput(Constants.JukeboxConstants.kNoteShooterPEChannel);

        /**
         * Rising (default): Debounces rising edges (transitions from false to true) only.
         * Falling: Debounces falling edges (transitions from true to false) only.
         * Both: Debounces all transitions.
         */
        _noteHolderPEDebouncer = new Debouncer(Constants.JukeboxConstants.kPhotoEyeDebounceTime,
                                               DebounceType.kRising);

        _noteShooterPEDebouncer = new Debouncer(Constants.JukeboxConstants.kPhotoEyeDebounceTime,
                                                DebounceType.kRising);
    }

    /**
     * Configures the Shooter Controllers and Profiling Constraints
     */
    private void configShooter() {
        // left shooter motor setup
        _shooterL = new CANSparkMax(Constants.JukeboxConstants.kShooterLeftMotorId,
                                    MotorType.kBrushless);

        _shooterL.setIdleMode(IdleMode.kCoast);
        _shooterL.setSmartCurrentLimit(Constants.JukeboxConstants.kHighStallLimit, Constants.JukeboxConstants.kFreeLimit);
        _shooterL.setInverted(true);
        _shooterL.burnFlash();

        // right shooter motor setup
        _shooterR = new CANSparkMax(Constants.JukeboxConstants.kShooterRightMotorId,
                                    MotorType.kBrushless);

        _shooterR.setIdleMode(IdleMode.kCoast);
        _shooterR.setSmartCurrentLimit(Constants.JukeboxConstants.kHighStallLimit, Constants.JukeboxConstants.kFreeLimit);
        _shooterR.setInverted(false);
        _shooterR.burnFlash();

        _shooterController = _shooterL.getPIDController();
        _shooterController.setP(Constants.JukeboxConstants.kShooterFeedForwardKp);
        _shooterController.setI(0);
        _shooterController.setD(0);
        _shooterController.setIZone(0);
        _shooterController.setFF(0);
        _shooterController.setOutputRange(0, 1.0);

        _shooterRController = _shooterR.getPIDController();
        _shooterRController.setP(Constants.JukeboxConstants.kShooterFeedForwardKp);
        _shooterRController.setI(0);
        _shooterRController.setD(0);
        _shooterRController.setIZone(0);
        _shooterRController.setFF(0);
        _shooterRController.setOutputRange(0, 1.0);

        // creates the feed foward for the shooter
        _shooterFeedForward = new SimpleMotorFeedforward(
            Constants.JukeboxConstants.kShooterFeedForwardKs,
            Constants.JukeboxConstants.kShooterFeedForwardKv
        );

        _shootReadyDebouncer = new Debouncer(.04, DebounceType.kRising);
        
        _manualShooterSpeed = 0.0;
        _shooterSetpoint = 0.0;
        _loopCounter = 0;
    }

    /**
     * Configures the Elevator Controllers and Profiling Constraints
     */
    private void configElevator() {
        // left elevator motor setup
        _elevatorL = new CANSparkMax(Constants.JukeboxConstants.kLElevatorId,
                                     MotorType.kBrushless);

        _elevatorL.setIdleMode(IdleMode.kBrake);
        _elevatorL.setSmartCurrentLimit(Constants.JukeboxConstants.kLowStallLimit, Constants.JukeboxConstants.kFreeLimit);
        _elevatorL.setInverted(true);
        
        // right elevator motor setup
        _elevatorR = new CANSparkMax(Constants.JukeboxConstants.kRElevatorId,
                                     MotorType.kBrushless);

        _elevatorR.setIdleMode(IdleMode.kBrake);
        _elevatorR.setSmartCurrentLimit(Constants.JukeboxConstants.kLowStallLimit, Constants.JukeboxConstants.kFreeLimit);
        _elevatorR.setInverted(false);
        _elevatorR.burnFlash();
        
        _elevatorController = _elevatorL.getPIDController();
        _elevatorController.setP(Constants.JukeboxConstants.kElevatorFeedForwardKp);
        _elevatorController.setI(0);
        _elevatorController.setD(Constants.JukeboxConstants.kCommonKd);
        _elevatorController.setIZone(0);
        _elevatorController.setFF(0);
        _elevatorController.setOutputRange(-1.0, 1.0);

        // creates the feed foward for the elevator
        _elevatorFeedForward = new ElevatorFeedforward(
            Constants.JukeboxConstants.kElevatorFeedForwardKs,
            Constants.JukeboxConstants.kElevatorFeedForwardKg,
            Constants.JukeboxConstants.kElevatorFeedForwardKv
        );

        Constraints elevatorProfileConstraints;

        // the constraints our elevator has
        elevatorProfileConstraints = new TrapezoidProfile.Constraints(
            Constants.JukeboxConstants.kElevatorMaxVelocity,
            Constants.JukeboxConstants.kElevatorMaxAcceleration
        );

        _elevatorProfile = new TrapezoidProfile(elevatorProfileConstraints);

        // our desired state and current state
        _elevatorProfileGoal = new TrapezoidProfile.State();
        _elevatorProfileSetpoint = new TrapezoidProfile.State();
        
        _manualElevatorSpeed = 0.0;
        
        _elevatorL.burnFlash();
    }
    
    /**
     * Configures the Shooter Angle Controllers and Profiling Constraints
     */
    private void configShooterAngle() {
        // shooter angle motor setup
        _shooterAngle = new CANSparkMax(Constants.JukeboxConstants.kShooterAngleId,
                                        MotorType.kBrushless);

        _shooterAngle.setIdleMode(IdleMode.kBrake);
        _shooterAngle.setSmartCurrentLimit(Constants.JukeboxConstants.kLowStallLimit, Constants.JukeboxConstants.kFreeLimit);
        _shooterAngle.setInverted(false);
       

        _shooterAngleController = _shooterAngle.getPIDController();
        _shooterAngleController.setP(Constants.JukeboxConstants.kShooterAngleFeedForwardKp);
        _shooterAngleController.setI(0);
        _shooterAngleController.setD(0.2);
        _shooterAngleController.setIZone(0);
        _shooterAngleController.setFF(0);
        _shooterAngleController.setOutputRange(-1.0, 1.0);
        
        // creates the feed foward for the shooter angle
        _shooterAngleFeedForward = new ArmFeedforward(
            Constants.JukeboxConstants.kShooterAngleFeedForwardKs,
            Constants.JukeboxConstants.kShooterAngleFeedForwardkG,
            Constants.JukeboxConstants.kShooterAngleFeedForwardKv
        );

        Constraints shooterAngleProfileConstraints;

        // the constraints our shooter angle has
        shooterAngleProfileConstraints = new TrapezoidProfile.Constraints(
            Constants.JukeboxConstants.kShooterAngleMaxVelocity,
            Constants.JukeboxConstants.kShooterAngleMaxAcceleration
        );

        _shooterAngleProfile = new TrapezoidProfile(
            shooterAngleProfileConstraints
        );

        // our desired state and current state
        _shooterAngleProfileGoal = new TrapezoidProfile.State();
        _shooterAngleProfileSetpoint = new TrapezoidProfile.State();

        _manualShooterAngleSpeed = 0.0;
         _shooterAngle.burnFlash();
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
        if (target == LocationTarget.AMP) {

            _feeder.set(-Constants.JukeboxConstants.kFeederShootSpeed);
        } else if (_previousState == JukeboxState.PREP_TRAP)
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
        } else if (_previousState == JukeboxState.PREP) {
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
        if (_previousState != JukeboxState.CLIMB && _previousState != JukeboxState.SCORE) return;
        if (_previousState == JukeboxState.SCORE) {
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

        _targetDistance = _drivetrain.getDistanceToSpeaker();

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
        if (_previousState != JukeboxState.EXTEND_FOR_CLIMB) {
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

    public boolean doneScoring() {
        return !hasNote() && _currentState == JukeboxState.SCORE;
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

    public void setTargetSpeaker() {
        target = LocationTarget.SPEAKER;
    }

    public void setTargetAmp() {
        target = LocationTarget.AMP;
    }

    public void setTargetZone() {
        target = LocationTarget.ZONE;
    }

    public void setNoTarget()
    {
        target = LocationTarget.NONE;
    }

    public LocationTarget getLocationTarget()
    {
        return target;
    }

    public Command handleCurrentState()
    {
        switch(_currentState) {
            case MANUAL:
                return run(() -> manual());
            case JUKEBOX_TEST:
                return run(() -> jukeboxTest());
            case SCORE:
                return run(() -> score());
            // case PREP_SPEAKER:
            //     return run(() -> prepSpeaker());
            // case PREP_SPEAKER_PODIUM:
            //     return run(() -> prepSpeakerPodium());
            // case PREP_SPEAKER_LINE:
            //     return run(() -> prepSpeakerLine());
            // case PREP_SPEAKER_SUBWOOFER:
            //     return run(() -> prepSpeakerSubwoofer());
            // case PREP_LAUNCH:
            //     return run(() -> prepLaunch());
            // case PREP_HUMAN_INTAKE:
            //     return run(() -> prepHumanIntake());
            // case PREP_AMP:
            //     return run(() -> prepAmp());
            case RESET:
                return run(() -> reset());
            case EJECT:
                return run(() -> emergencyEject());
            case EXTEND_FOR_CLIMB:
                return run(() -> extendForClimb());
            case CLIMB:
                return run(() -> climb());
            case PREP:
                if (target == LocationTarget.SPEAKER) {
                    return run(() -> prepSpeaker());
                } else if (target == LocationTarget.AMP) {
                    return run(() -> prepAmp());
                } else if (target == LocationTarget.ZONE) {
                    return run(() -> prepLaunch());
                }
            default:
                return run(() -> idle());
        }

        // switch (_currentState) {
        //     case MANUAL:
        //     case CLIMB:
        //         break;
        //     // case EXTEND_FOR_CLIMB:
        //     //     handleShooterSpeed();
        //     //     handleShooterAnglePosition();
        //     //     break;
        //     default:
        //         handleShooterSpeed();
        //         handleShooterAnglePosition();
        //         handleElevatorPosition();
        //         break;
        // }
    }

    @Override
    public void periodic()
    {
        logTelemetry();
        handleCurrentState().schedule();
        if(_currentState != JukeboxState.MANUAL || _currentState != JukeboxState.CLIMB || _currentState != JukeboxState.EXTEND_FOR_CLIMB) {
            handleShooterSpeed();
            handleShooterAnglePosition();
            handleElevatorPosition();
        } else if (_currentState == JukeboxState.EXTEND_FOR_CLIMB) {
            handleShooterSpeed();
            handleShooterAnglePosition();
        }
    }

    @Override
    public InstantCommand setWantedState(JukeboxState state)
    {
        return new InstantCommand(() -> {
            // checks to see if the current state is what we are trying to set the state to
        if (state != _currentState) {
            switch (_currentState) 
            {
                case CLIMB: 
                    if (state != JukeboxState.EXTEND_FOR_CLIMB &&
                        state != JukeboxState.PREP_TRAP) {
                        
                        return;
                    }
                    if (state == JukeboxState.EXTEND_FOR_CLIMB) {
                        
                        _elevatorProfileSetpoint = new TrapezoidProfile.State(getElevatorPosition(), 0);
                    }
                    break;

                default:
                    break;            
            }

            _previousState = _currentState;
            _currentState = state;
        }
        }, this);
    }

    public boolean getIsReadyToScore() {
        return _isReadyToShoot;
    }

    public boolean isReadyToScore() {
        switch (_currentState) {
            case PREP_AMP:
            case PREP_TRAP:
                // Logic if Prep Amp/Trap is ready to score
                break;
            case PREP_SPEAKER:
            case PREP_SPEAKER_PODIUM:
            case PREP_SPEAKER_SUBWOOFER:
            case PREP_SPEAKER_LINE:
                // Error is the absolute value of the difference between Target and Actual 
                var shooterError = Math.abs((_shooterSetpointRpm + 300) - _shooterL.getEncoder().getVelocity());
                var angleError = Math.abs(_shooterAngleProfileSetpoint.position - _shooterAngle.getEncoder().getPosition());

                // TODO: These error numbers need to tuned/configured. 
                // We also may want a debouncer for the result of this method so that it must be ready to score for a minimum amount of time first.
                return ((shooterError <= 100 || _shooterL.getEncoder().getVelocity() >= 4200) && angleError <= .0175);
            default:
                return false;
        }

        return false;
    }

    public JukeboxState getJukeboxState()
    {
        return _currentState;
    }



    public void logTelemetry() {
        _dashboardShooterTargetAngle = SmartDashboard.getNumber("dashboardShooterTargetAngle", 0.0);
        _dashboardShooterTargetSpeed = SmartDashboard.getNumber("dashboardShooterTargetSpeed", 0.0);
        _dashboardShooterRPercent = SmartDashboard.getNumber("dashboardShooterRPercent", 0.85);
        SmartDashboard.putNumber("Elevator motor left enconder position", _elevatorL.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator motor left enconder velocity", _elevatorL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Elevator motor left temp", _elevatorL.getMotorTemperature());
        SmartDashboard.putNumber("Elevator motor left speed", _elevatorL.get());

        SmartDashboard.putNumber("Elevator motor right enconder position", _elevatorR.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator motor right enconder velocity", _elevatorR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Elevator motor right temp", _elevatorR.getMotorTemperature());
        SmartDashboard.putNumber("Elevator motor right speed", _elevatorR.get());

        SmartDashboard.putNumber("Feeder motor enconder position", _feeder.getEncoder().getPosition());
        SmartDashboard.putNumber("Feeder motor enconder velocity", _feeder.getEncoder().getVelocity());
        SmartDashboard.putNumber("Feeder motor temp", _feeder.getMotorTemperature());
        SmartDashboard.putNumber("Feeder motor speed", _feeder.get());
        
        SmartDashboard.putNumber("Shooter angle motor enconder position", _shooterAngle.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter angle motor enconder velocity", _shooterAngle.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter angle motor temp", _shooterAngle.getMotorTemperature());
        SmartDashboard.putNumber("Shooter angle motor speed", _shooterAngle.get());

        SmartDashboard.putNumber("Shooter motor left enconder position", _shooterL.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter motor left enconder velocity", _shooterL.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter motor left temp", _shooterL.getMotorTemperature());
        SmartDashboard.putNumber("Shooter motor left speed", _shooterL.get());

        SmartDashboard.putNumber("Shooter motor right enconder position", _shooterR.getEncoder().getPosition());
        SmartDashboard.putNumber("Shooter motor right enconder velocity", _shooterR.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter motor right  temp", _shooterR.getMotorTemperature());
        SmartDashboard.putNumber("Shooter motor right speed", _shooterR.get());

        SmartDashboard.putBoolean("is the note pass the shooter limit switch", _noteShooterPE.get());
        SmartDashboard.putBoolean("is the note in the correct position in the holder", _noteHolderPE.get());

        SmartDashboard.putString("Jukebox current state", _currentState.toString());
        SmartDashboard.putString("Jukebox previous state", _previousState.toString());
        SmartDashboard.putBoolean("isholding", !hasNote());


        _inNoteHolder = !_noteHolderPEDebouncer.calculate(
            _noteHolderPE.get()
        );

        _inNoteShooter = !_noteShooterPEDebouncer.calculate(
            _noteShooterPE.get()
        );

        _isReadyToShoot = _shootReadyDebouncer.calculate(isReadyToScore());

        SmartDashboard.putNumber("dashboardShooterTargetAngle", _dashboardShooterTargetAngle);
        SmartDashboard.putNumber("dashboardShooterTargetSpeed", _dashboardShooterTargetSpeed);
        SmartDashboard.putNumber("dashboardShooterRPercent", _dashboardShooterRPercent);
        SmartDashboard.putBoolean("shooterReady", isReadyToScore());
        // SmartDashboard.putNumber("Vision system distance", _vision.getDistance());
    }
}
