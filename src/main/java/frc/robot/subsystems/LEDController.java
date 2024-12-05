package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.JukeboxState;

public class LEDController extends SubsystemBase {

    private static LEDController _instance;

    private CANdle upperCandle;
    private CANdle lowerCandle;

    private CANdleConfiguration config;

    private Optional<Alliance> _alliance;

    // Animation for Jukebox state
    private Animation SCORE_LIGHTS;
    private Animation PREP_SPEAKER_LIGHTS;
    private Animation PREP_AMP_LIGHTS;
    private Animation PREP_TRAP_LIGHTS;
    private Animation RESET_LIGHTS;
    private Animation EXTEND_FOR_CLIMB_LIGHTS;    
    private Animation CLIMB_LIGHTS;
    private Animation MANUAL_LIGHTS;
    private Animation IDLE_LIGHTS;
    private Animation FEED_LIGHTS;

    //Location State
    private Animation LOCATION_SPEAKER;
    private Animation LOCATION_AMP;
    private Animation LOCATION_ZONE;
    private Animation LOCATION_NONE;


    private int underNumLeds;
    private int jukeboxNumLeds;

    private Jukebox _jukebox;

    /**
     * They are grb lights
     */
    public LEDController(Jukebox jukebox)
    {
        _jukebox = jukebox;
        upperCandle = new CANdle(Constants.LEDConstants.kUpperCandle);
        //lowerCandle = new CANdle(Constants.LEDConstants.kLowerCandle);
        // underNumLeds = 400;
        upperCandle.setLEDs(0, 255, 0, 0, 0, jukeboxNumLeds);
        jukeboxNumLeds = 50;        
        // animation for IDLE --color is Pure White
        IDLE_LIGHTS = new StrobeAnimation(255, 255, 0, 0, .15, jukeboxNumLeds);
        // animation for SCORE --color is Green
        SCORE_LIGHTS = new ColorFlowAnimation(0, 255, 0, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for PREP_SPEAKER --color is Turquoise
        PREP_SPEAKER_LIGHTS = new FireAnimation(.5, .5, _jukebox.getShooterLeds(jukeboxNumLeds), .25, .1);
        // animation for PREP_AMP --color is Indigo
        PREP_AMP_LIGHTS = new RainbowAnimation(.5, .5, jukeboxNumLeds);
        // animation for PREP_TRAP --color is Sunset Orange
        PREP_TRAP_LIGHTS = new ColorFlowAnimation(255, 165, 0, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for RESET --color is Magenta
        RESET_LIGHTS = new ColorFlowAnimation(255, 0, 255, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for EXTEND_FOR_CLIMB --color is Lavender
        EXTEND_FOR_CLIMB_LIGHTS = new ColorFlowAnimation(230, 230, 250, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for CLIMB --color is Gold
        CLIMB_LIGHTS = new ColorFlowAnimation(255, 215, 0, 0, 0.5, jukeboxNumLeds, Direction.Forward);
        // animation for MANUAL --color is Purple
        MANUAL_LIGHTS = new ColorFlowAnimation(128, 0, 128, 0, 0.5, jukeboxNumLeds, Direction.Forward);

        LOCATION_SPEAKER = new ColorFlowAnimation(0, 0, 255);
        //blue

        LOCATION_AMP = new ColorFlowAnimation(255, 0 , 0);
        //red

        LOCATION_ZONE = new ColorFlowAnimation(255,105,180);
        //pink

        LOCATION_NONE = new ColorFlowAnimation(0, 255, 0);
        //green


    }


    public void colorTest()
    {
        upperCandle.configBrightnessScalar(1);
        upperCandle.setLEDs(255, 255, 255);
    }
    
    /**
     * Turns the brightness of the candle to 0 basically off.
     */
    public void off()
    {
        config.brightnessScalar = 0;
    }

    /**
     * For each state in JukeBox a light will come on
     */
    public void handleLights()
    {
        // _alliance = DriverStation.getAlliance();
        // if (_alliance.isPresent() && _alliance.get() == DriverStation.Alliance.Blue) {
        //     lowerCandle.setLEDs(0, 0, 255, 0, 0, jukeboxNumLeds); 
        // } else {
        //     lowerCandle.setLEDs(255, 0, 0, 0, 0, jukeboxNumLeds); 
        // }

        var currentState = _jukebox.getJukeboxState();
        var targetState = _jukebox.getLocationTarget();
        if(!_jukebox.hasNote())
        {
            upperCandle.animate(IDLE_LIGHTS);

        }
        else
        {
            upperCandle.clearAnimation(0);
            switch(targetState)
            {
                case SPEAKER:
                upperCandle.setLEDs(0, 0, 255);
                //blue
                break;
                case AMP:
                upperCandle.setLEDs(255, 0, 0);
                //red
                break;
                case ZONE:
                upperCandle.setLEDs(255,105,180);
                //pink
                break;
                default:
                upperCandle.setLEDs(0, 255, 0);
                //green (none)
                break;
            }
        }
            
    }   
    @Override
    public void periodic()
    {
        //handleLights();
    }
}
