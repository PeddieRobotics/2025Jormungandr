package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.IntakeConstants;

public class Lights extends SubsystemBase {
    private enum LightState {
        IDLE,
        ALGAE_INTAKING,
        CORAL_INTAKING,
        INTAKEN,
        CLIMBING,
        DONE_CLIMBING,
        HAS_TARGET,
        ALIGNED,
        FAILED
    }

    public static Lights lights;

    private final CANdle candle;
    private double lastIntaked;
    private boolean isClimbing;
    private LightState systemState, requestedSystemState;
    
    private PVBack pvBack;
    private PVFrontLeft pvFrontLeft;
    private PVFrontMiddle pvFrontMiddle;
    private PVFrontRight pvFrontRight;
    private PVLeft pvLeft;

    public static Lights getInstance() {
        if (lights == null)
            lights = new Lights();
        return lights;
    }

    public Lights() {
        candle = new CANdle(RobotMap.CANDLE_ID);
        
        // pvBack = PVBack.getInstance();
        pvFrontLeft = PVFrontLeft.getInstance();
        pvFrontMiddle = PVFrontMiddle.getInstance();
        pvFrontRight = PVFrontRight.getInstance();
        // pvLeft = PVLeft.getInstance();
    }

    public void requestState(LightState request) {
        candle.clearAnimation(0);
        candle.setLEDs(0, 0, 0);

        if (request == LightState.INTAKEN)
            lastIntaked = Timer.getFPGATimestamp();
        
        requestedSystemState = request;

        // TODO: used switch statements last year but it seems redundant; revisit if something is wrong
        // switch (request) {
        //     case INTAKEN:
        //         lastIntaked = Timer.getFPGATimestamp();
        //         break;
        //     case HAS_TARGET:
        //         candle.setLEDs(0, 0, 255);
        //         break;
        //     case ALIGNED:
        //         candle.setLEDs(0, 255, 0);
        //         break;
        //     case FAILED:
        //         candle.setLEDs(255, 0, 0);
        //         break;
        //     default:
        //         break;
        // }
    }

    @Override
    public void periodic() {
        systemState = requestedSystemState;

        switch (systemState) {
            case IDLE:
                break;

            case ALGAE_INTAKING:
                // aqua color
                candle.setLEDs(0, 128, 128);
                break;

            case CORAL_INTAKING:
                // orange color
                candle.setLEDs(255, 127, 80); // spsd to be a coral color
                break;

            case INTAKEN:
                candle.animate(new StrobeAnimation(0, 255, 0, 0, 0.3, 8), 0);

                // TODO: consider removing if it interferes with pose estimation
                if (!DriverStation.isAutonomous())
                    setBlinking(true);
                if (Timer.getFPGATimestamp() - lastIntaked >= 1.5)
                    setBlinking(false);

                break;

            case CLIMBING:
                candle.animate(new ColorFlowAnimation(255, 0, 255, 0, 0.3, 8, Direction.Forward), 0);
                break;

            case DONE_CLIMBING:
                candle.animate(new RainbowAnimation(), 0);
                break;

            case HAS_TARGET:
                candle.setLEDs(0, 0, 255);
                break;

            case ALIGNED:
                candle.animate(new RainbowAnimation(), 0);
                break;

            case FAILED:
                candle.setLEDs(255, 0, 0);
                break;

            default:
                break;
        }
    }

    public LightState getLightState() {
        return systemState;
    }
    
    private void setBlinking(boolean blinking) {
        pvBack.setBlinking(blinking);
        pvFrontLeft.setBlinking(blinking);
        pvFrontMiddle.setBlinking(blinking);
        pvFrontRight.setBlinking(blinking);
        pvLeft.setBlinking(blinking);
    }
}
