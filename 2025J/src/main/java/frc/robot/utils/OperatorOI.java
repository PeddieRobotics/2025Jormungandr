package frc.robot.utils;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.ManualElevatorControl;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class OperatorOI {

    private static OperatorOI instance;
    private Superstructure superstructure;
    private PS4Controller controller;

    public static OperatorOI getInstance() {
        if (instance == null) {
            instance = new OperatorOI();
        }
        return instance;
    }

    public OperatorOI() {
        superstructure = Superstructure.getInstance();
        configureController();
        SmartDashboard.putBoolean("L3 Held?", false);
    }

    public void configureController() {

        // READ: Press FN + X on the PS5 edge controller to activate the 2025 binding
        // profile

        controller = new PS4Controller(1);

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.L1_PREP)));

        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.L2_PREP)));

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.L3_PREP)));

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        triangleButton.onTrue(new InstantCommand(() -> {
            if (superstructure.getCurrentState() == SuperstructureState.L4_PRESTAGE) {
                superstructure.requestState(SuperstructureState.L4_PREP);
            } else {
                superstructure.requestState(SuperstructureState.L4_PRESTAGE);
            }
        }));

        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));


        Trigger muteButton = new JoystickButton(controller, 15);
        // enter climb mode

        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        // L1Bumper.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.BARGE_PREP)));

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        // R1Bumper.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.L4_PREP)));

        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);
        L2Trigger.whileTrue(new ManualElevatorControl());

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);
        R2Trigger.whileTrue(new ManualArmControl());

        Trigger L3Trigger = new JoystickButton(controller, PS4Controller.Button.kL3.value);

        Trigger R3Trigger = new JoystickButton(controller, PS4Controller.Button.kR3.value);

        Trigger ps5Button = new JoystickButton(controller, PS4Controller.Button.kPS.value);

        Trigger dpadUpTrigger = new Trigger(() -> controller.getPOV() == 0);
        dpadUpTrigger.onTrue(new InstantCommand(() -> {
            if (superstructure.getCurrentState() == SuperstructureState.BARGE_PRESTAGE) {
                superstructure.requestState(SuperstructureState.BARGE_PREP);
            } else {
                superstructure.requestState(SuperstructureState.BARGE_PRESTAGE);
            }
        }));

        Trigger dpadLeftTrigger = new Trigger(() -> controller.getPOV() == 270);
        dpadLeftTrigger.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.REEF1_ALGAE_INTAKE)));

        Trigger dpadRightTrigger = new Trigger(() -> controller.getPOV() == 90);
        dpadRightTrigger.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.REEF2_ALGAE_INTAKE)));

        Trigger dpadDownTrigger = new Trigger(() -> controller.getPOV() == 180);
        dpadDownTrigger.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.PROCESSOR_PREP)));

        Trigger optionButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);
        // TODO: home elevator
        
        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);
        // TODO: home arm

    }

    public double getForward() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public double getRightForward() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kRightY.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public boolean bothBumpersHeld() {
        return controller.getL1Button() && controller.getR1Button();
    }

    public boolean getLeftBumperHeld(){
        return controller.getL1Button();
    }

    public boolean getRightBumperHeld(){
        return controller.getR1Button();
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
        SmartDashboard.putBoolean("L3 Held?", controller.getL3Button());
        return controller.getL3Button();
    }

    public boolean R3Held() {
        return controller.getR3Button();
    }

    public boolean dPadDownHeld() {
        return controller.getPOV() == 180;
    }

}