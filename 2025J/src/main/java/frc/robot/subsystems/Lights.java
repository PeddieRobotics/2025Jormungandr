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

    public static Lights getInstance() {
        if (lights == null)
            lights = new Lights();
        return lights;
    }

    public Lights() {
        candle = new CANdle(RobotMap.CANDLE_ID);
    }

    public void requestState(LightState request) {
        candle.clearAnimation(0);
        candle.setLEDs(0,0,0);

        if (request == LightState.INTAKEN)
            lastIntaked = Timer.getFPGATimestamp();
        
        requestedSystemState = request;
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
        //used switch statements last year but it seems redundant; revisit if something is wrong
    }

    @Override
    public void periodic() {
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

                if (!DriverStation.isAutonomous()) {
                    // TODO: photonvision
                    //don't do this if it'll mess w/ alignment
                    // LimelightHelpers.setLEDMode_ForceBlink("limelight-left"); //change based on limelight names
                    // LimelightHelpers.setLEDMode_ForceBlink("limelight-right");
                    // LimelightHelpers.setLEDMode_ForceBlink("limelight-back");
                }

                if(Timer.getFPGATimestamp() - lastIntaked > 1.5) {
                    //TODO: verify that there is no additional limelight logic needed here
                    requestState(LightState.IDLE);
                }

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
}
