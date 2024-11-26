package frc.robot.utilities;

import java.util.Optional;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.enums.JukeboxState;
import frc.robot.subsystems.Jukebox;

public class LEDController {

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

    private int underNumLeds;
    private int jukeboxNumLeds;

    private Jukebox _jukebox;

    /**
     * They are grb lights
     */

    LEDController()
    {
        upperCandle = new CANdle(Constants.LEDConstants.kUpperCandle);
        lowerCandle = new CANdle(Constants.LEDConstants.kLowerCandle);
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
        _alliance = DriverStation.getAlliance();
        if (_alliance.isPresent() && _alliance.get() == DriverStation.Alliance.Blue) {
            lowerCandle.setLEDs(0, 0, 255, 0, 0, jukeboxNumLeds); 
        } else {
            lowerCandle.setLEDs(255, 0, 0, 0, 0, jukeboxNumLeds); 
        }

        var currentState = _jukebox.getJukeboxState();
        switch (currentState) {
            case IDLE:
                if (_jukebox.hasNote())
                {
                    upperCandle.clearAnimation(0);
                    upperCandle.setLEDs(0, 255, 0, 0, 0, jukeboxNumLeds);
                } else {
                    upperCandle.animate(IDLE_LIGHTS);
                }
                break;
            case SCORE:
                upperCandle.setLEDs(0, 255, 255, 0, 0, jukeboxNumLeds);
                break;
            case PREP_LAUNCH:
                upperCandle.setLEDs(170, 51, 106, 0, 0, _jukebox.getShooterLeds(jukeboxNumLeds));
                break;
            case PREP_SPEAKER_PODIUM:
            case PREP_SPEAKER_LINE:
            case PREP_SPEAKER:
                // upperCandle.clearAnimation(0);
                // upperCandle.animate(PREP_SPEAKER_LIGHTS);
                upperCandle.setLEDs(255, 0, 0, 0, 0, _jukebox.getShooterLeds(jukeboxNumLeds));
                break;
            case PREP_AMP:
                // upperCandle.clearAnimation(0);
                // upperCandle.animate(PREP_AMP_LIGHTS);
                upperCandle.setLEDs(255, 153, 51, 0, 0, jukeboxNumLeds);
                break;
            case PREP_TRAP:
                upperCandle.clearAnimation(0);
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            case RESET:
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            case EXTEND_FOR_CLIMB:
                upperCandle.clearAnimation(0);
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            case CLIMB:
                if (_jukebox.getElevatorPosition() < .05) {
                    upperCandle.animate(CLIMB_LIGHTS);
                }
                break;
            case MANUAL:
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            default:
                upperCandle.setLEDs(0, 255, 62, 0, 0, jukeboxNumLeds);
                break;
            }


        
    }
}
