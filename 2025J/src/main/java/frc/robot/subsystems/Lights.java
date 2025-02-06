package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Lights extends SubsystemBase{
    public static Lights lights;
    private final CANdle candle;
    private double lastIntook;
    private boolean isClimbing;

    public enum LightState{
        IDLE,
        ALGAE_INTAKING,
        CORAL_INTAKING,
        INTOOK,
        CLIMBING,
        DONE_CLIMBING,
        HAS_TARGET,
        ALIGNED,
        FAILED
    }

    LightState systemState;
    LightState requestedSystemState;

    public Lights(){
        candle = new CANdle(0);
    }

    public static Lights getInstance(){
        if(lights == null){
            lights = new Lights();
        }
        return lights;
    }

    public void requestState(LightState request){
        candle.clearAnimation(0);
        candle.setLEDs(0,0,0);

        switch (request){
            case INTOOK:
                lastIntook = Timer.getFPGATimestamp();
                break;
            case HAS_TARGET:
                candle.setLEDs(0, 0, 255);
                break;
            case ALIGNED:
                candle.setLEDs(0, 255, 0);
                break;
            case FAILED:
                candle.setLEDs(255, 0, 0);
                break;
            default:
                break;
        }
    }

    @Override
    public void periodic(){
        switch (systemState){
            case IDLE:
                break;
            case ALGAE_INTAKING:
                candle.setLEDs(0, 128, 128);
                break;
            case CORAL_INTAKING:
                candle.setLEDs(255, 255, 255);
                break;
            case INTOOK:
                lastIntook = Timer.getFPGATimestamp();
                break;
            case CLIMBING:
                break;
            case DONE_CLIMBING:
                candle.animate(new RainbowAnimation(),0);
                break;
            case HAS_TARGET:
                break;
            case ALIGNED:
                break;
            case FAILED:
                candle.setLEDs(255, 0, 0);
                break;
            default:
                break;
        }
    }

    public String stateAsString(){
        switch (systemState){
            case IDLE:
                return "IDLE";
            case ALGAE_INTAKING:
                return "ALGAE_INTAKING";
            case CORAL_INTAKING:
                return "CORAL_INTAKING";
            case INTOOK:
                return "INTOOK";
            case CLIMBING:
                return "CLIMBING";
            case DONE_CLIMBING:
                return "DONE_CLIMBING";
            case HAS_TARGET:
                return "HAS_TARGET";
            case ALIGNED:
                return "ALIGNED";
            case FAILED:
                return "FAILED";
        }
        return "";
    }
}
