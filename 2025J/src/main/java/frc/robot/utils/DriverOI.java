package frc.robot.utils;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.ReefCommands.AlignToReefOdometry;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PVFrontMiddle;
import frc.robot.utils.Constants.DriveConstants;

public class DriverOI {

    private static DriverOI instance;
    private Superstructure superstructure;
    private PS4Controller controller;

    public DriverOI() {
        controller = new PS4Controller(0);

        //superstructure = Superstructure.getInstance();
        configureController();

    }

    public static DriverOI getInstance() {
        if (instance == null) {
            instance = new DriverOI();
        }
        return instance;
    }

    public void configureController() {

        // READ: Press FN + X on the PS5 edge controller to activate the 2025 binding
        // profile

        controller = new PS4Controller(0);

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));

        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.HP_INTAKE)));

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        triangleButton.onTrue(new InstantCommand(() -> superstructure.sendToScore()));

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        
        
        Trigger muteButton = new JoystickButton(controller, 15);
        // TODO: add sensor condition
        //muteButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.EJECT_CORAL)));
        //muteButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.EJECT_ALGAE)));
        
        Trigger PSButton = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        PSButton.onTrue(new InstantCommand(() -> Drivetrain.getInstance().resetGyro()));

        // Set to climb
        // Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);

        // Trigger muteButton = new JoystickButton(controller, 15);
        // muteButton.onTrue(new InstantCommand(() -> {
        //     if (OperatorOI.getInstance().L3Held()) {
        //         superstructure.requestState(SuperstructureState.EJECT_ALGAE);
        //     } else {
        //         superstructure.requestState(SuperstructureState.EJECT_CORAL);
        //     }
        // }));


        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        // TODO: align left if coral, else align HP

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        // TODO: align right if coral, else align HP

        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        Trigger L3Trigger = new JoystickButton(controller, PS4Controller.Button.kL3.value);

        Trigger R3Trigger = new JoystickButton(controller, PS4Controller.Button.kR3.value);

        Trigger optionButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);
        optionButton.onTrue(new InstantCommand(() -> Drivetrain.getInstance().resetTranslation(PVFrontMiddle.getInstance().getEstimatedPose().getTranslation())));
        
        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);
        
        //TODO: right back button: orbit reef command
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

    public double getForward() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public double getStrafe() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public Translation2d getSwerveTranslation() {
        double xSpeed = getForward();
        double ySpeed = getStrafe();

        return new Translation2d(DriveConstants.kMaxFloorSpeed * xSpeed, DriveConstants.kMaxFloorSpeed * ySpeed);
    }

    public double getRotation() {
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation = (rightRotation - leftRotation) / 2.0;

        return Math.abs(combinedRotation) < 0.1 ? 0 : combinedRotation * DriveConstants.kMaxRotationSpeed; // TODO: convert to rad/s
    }
}
