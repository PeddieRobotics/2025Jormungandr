package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class DriverOI {

    private static DriverOI instance;
    private Superstructure superstructure;
    private OperatorOI operatorOI;
    private PS4Controller controller;
    private BooleanSupplier algaeMode; // fix this while flushing operator oi pleaseeeee :) :) :) :) :)

    public static DriverOI getInstance() {
        if (instance == null) {
            instance = new DriverOI();
        }
        return instance;
    }

    public DriverOI() {
        superstructure = Superstructure.getInstance();
        operatorOI = OperatorOI.getInstance();
    }

    public void configureController() {


        // READ: Press FN + X on the PS5 edge controller to activate the 2025 binding profile


        controller = new PS4Controller(0);

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);

        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);

        // Set to STOW state
        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));

        Trigger muteButton = new JoystickButton(controller, 15);
        muteButton.onTrue(new InstantCommand(() -> {
            if (operatorOI.L3Held()) {
                superstructure.requestState(SuperstructureState.EJECT_ALGAE);
            } else {
                superstructure.requestState(SuperstructureState.EJECT_CORAL);
            }
        }));

        // Set to GROUND_INTAKE (Coral or Algae)
        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        // L1Bumper.whileTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.CORAL_GROUND_INTAKE)));
        L1Bumper.onTrue(new ConditionalCommand(new InstantCommand(() -> superstructure.requestState(SuperstructureState.ALGAE_GROUND_INTAKE)),
                            new InstantCommand(() -> superstructure.requestState(SuperstructureState.CORAL_GROUND_INTAKE)),
                            algaeMode)); 

        // Send to score
        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        //R1Bumper.onTrue(new InstantCommand(() -> superstructure.sendToScore()));
        
        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        // Set to HP_INTAKE
        Trigger L3Trigger = new JoystickButton(controller, PS4Controller.Button.kL3.value);
        L3Trigger.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.HP_INTAKE)));

        // Set to CLIMB
        Trigger R3Trigger = new JoystickButton(controller, PS4Controller.Button.kR3.value);
        R3Trigger.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.CLIMB)));

        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);

        Trigger optionButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);

        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);

        Trigger dpadUpTrigger = new Trigger(() -> controller.getPOV() == 0);

        Trigger dpadLeftTrigger = new Trigger(() -> controller.getPOV() == 270);

        Trigger dpadRightTrigger = new Trigger(() -> controller.getPOV() == 90);

        Trigger dpadDownTrigger = new Trigger(() -> controller.getPOV() == 180);
    }

    public boolean bothBumpersHeld() {
        return controller.getL1Button() && controller.getR1Button();
    }

    public boolean bothTriggersHeld() {
        return leftTriggerHeld() & rightTriggerHeld();
    }

    public boolean leftTriggerHeld() {
        return controller.getL2Button();
    }

    public boolean onlyLeftTriggerHeld() {
        return leftTriggerHeld() && !rightTriggerHeld();
    }

    public boolean rightTriggerHeld() {
        return controller.getR2Button();
    }

    public boolean onlyRightTriggerHeld() {
        return !leftTriggerHeld() && rightTriggerHeld();
    }

    public boolean onlyOneTriggerHeld() {
        return leftTriggerHeld() ^ rightTriggerHeld();
    }

    public boolean L3Held() {
        return controller.getL3Button();
    }

    public boolean R3Held() {
        return controller.getR3Button();
    }

    public boolean dPadDownHeld() {
        return controller.getPOV() == 180;
    }

    // public double getLeftForward() {
    //     double input = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
    //     if (Math.abs(input) < OIConstants.kDrivingDeadband) {
    //         input = 0;
    //     } else {
    //         input *= 0.7777;
    //     }
    //     return input;
    // }

    // public double getRightForward() {
    //     double input = -controller.getRawAxis(PS4Controller.Axis.kRightY.value);
    //     if (Math.abs(input) < OIConstants.kDrivingDeadband) {
    //         input = 0;
    //     } else {
    //         input *= 0.7777;
    //     }
    //     return input;
    // }
    
}