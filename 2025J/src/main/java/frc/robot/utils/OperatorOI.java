package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class OperatorOI {

    private static OperatorOI instance;
    private Superstructure superstructure;
    private DriverOI driverOI;
    private PS4Controller controller;

    public static OperatorOI getInstance() {
        if (instance == null) {
            instance = new OperatorOI();
        }
        return instance;
    }

    public OperatorOI() {
        superstructure = Superstructure.getInstance();
        driverOI = DriverOI.getInstance();
        configureController();
    }

    public void configureController() {


        // READ: Press FN + X on the PS5 edge controller to activate the 2025 binding profile


        controller = new PS4Controller(1);

        // If L3 held, sets to PROCESSOR_PREP --- If L3 is NOT held, sets to L1_PREP
        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new InstantCommand(() -> {
            if (L3Held()) {
                superstructure.requestState(SuperstructureState.PROCESSOR_PREP);
            } else {
                superstructure.requestState(SuperstructureState.L1_PREP);
            }
        }));

        // If L3 held, sets to REEF1_INTAKE --- If L3 is NOT held, sets to L2_PREP
        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new InstantCommand(() -> {
            if (L3Held()) {
                superstructure.requestState(SuperstructureState.REEF1_INTAKE);
            } else {
                superstructure.requestState(SuperstructureState.L2_PREP);
            }
        }));

        // If L3 held, sets to BARGE_PREP--- If L3 is NOT held, sets to L4_PREP
        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        triangleButton.onTrue(new InstantCommand(() -> {
            if (L3Held()) {
                superstructure.requestState(SuperstructureState.BARGE_PREP);
            } else {
                superstructure.requestState(SuperstructureState.L4_PREP);
            }
        }));

        // If L3 held, sets to REEF2_INTAKE --- If L3 is NOT held, sets to L3_PREP
        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.onTrue(new InstantCommand(() -> {
            if (L3Held()) {
                superstructure.requestState(SuperstructureState.REEF2_INTAKE);
            } else {
                superstructure.requestState(SuperstructureState.L3_PREP);
            }
        }));

        // Set to STOW state
        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));

        Trigger muteButton = new JoystickButton(controller, 15);
        muteButton.onTrue(new InstantCommand(() -> {
            if (L3Held()) {
                superstructure.requestState(SuperstructureState.EJECT_ALGAE);
            } else {
                superstructure.requestState(SuperstructureState.EJECT_CORAL);
            }
        }));

        // Set to CLIMB_PREP
        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        //L1Bumper.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.CLIMB_PREP)));

        //Set to CLIMB
        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        //R1Bumper.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.CLIMB)));
        
        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        Trigger L3Trigger = new JoystickButton(controller, PS4Controller.Button.kL3.value);

        Trigger R3Trigger = new JoystickButton(controller, PS4Controller.Button.kR3.value);

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