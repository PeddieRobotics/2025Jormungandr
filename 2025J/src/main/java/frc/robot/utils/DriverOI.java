package frc.robot.utils;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Superstructure.SuperstructureState;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.ReefCommands.AlignToReef2D;
import frc.robot.commands.ReefCommands.AlignToReef2D.AlignmentDestination;
import frc.robot.commands.ReefCommands.AlignToReefEstimatedPose;
import frc.robot.commands.ScoreCommands.AlignAndScore;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.utils.Constants.DriveConstants;

public class DriverOI {

    private static DriverOI instance;
    private Superstructure superstructure;
    private PS4Controller controller;

    public DriverOI() {
        controller = new PS4Controller(0);

        superstructure = Superstructure.getInstance();
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

        Trigger PSButton = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        PSButton.onTrue(new InstantCommand(() -> Drivetrain.getInstance().resetGyro()));

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));

        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.HP_INTAKE)));

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        // squareButton.onTrue(new InstantCommand(() -> CalculateReefTarget.calculateTargetID()));
        squareButton.whileTrue(new AlignToReefEstimatedPose(0));

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        // triangleButton.onTrue(new AlignAndScore(true)); //right align
        triangleButton.onTrue(new InstantCommand(() -> superstructure.sendToScore()));
        // Claw.getInstance().stopClaw()));

        Trigger muteButton = new JoystickButton(controller, 15);
        // Set to climb
        muteButton.onTrue(new InstantCommand(() -> {
            Drivetrain.getInstance().resetTranslation(LimelightFrontMiddle.getInstance().getEstimatedPoseMT2().get().getTranslation());
        }));

        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new InstantCommand(() -> {
            SmartDashboard.putBoolean("is inside bad hexagon", CalculateReefTarget.insideBadHexagon(Drivetrain.getInstance().getPose()));
        }));

        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        // TODO: align left if coral, else align HP
        // L1Bumper.whileTrue(new AlignToReef2D(AlignmentDestination.LEFT));
        L1Bumper.whileTrue(new AlignToReefEstimatedPose(0.1651));

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        // TODO: align right if coral, else align HP
        // R1Bumper.whileTrue(new AlignToReef2D(AlignmentDestination.RIGHT));
        R1Bumper.whileTrue(new AlignToReefEstimatedPose(-0.1651));

        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        Trigger L3Trigger = new JoystickButton(controller, PS4Controller.Button.kL3.value);


        Trigger R3Trigger = new JoystickButton(controller, PS4Controller.Button.kR3.value);
        //R3Trigger.onTrue(new OrbitReef());
  

        Trigger optionButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);
        optionButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.EJECT_CORAL)));

        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);
        shareButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.EJECT_ALGAE)));
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

    public double getRightForward() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kRightY.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public double getStrafe() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public Translation2d getSwerveTranslation() {
        double xSpeed = getForward();
        double ySpeed = getStrafe();

        double xSpeedCommanded;
        double ySpeedCommanded;

        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;

        Translation2d next_translation = new Translation2d(xSpeedCommanded, ySpeedCommanded);

        double norm = next_translation.getNorm();
        if (norm < DriveConstants.kDrivingDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(next_translation.getX(), next_translation.getY());
            Translation2d deadband_vector = fromPolar(deadband_direction, DriveConstants.kDrivingDeadband);

            double new_translation_x = next_translation.getX()
                    - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double new_translation_y = next_translation.getY()
                    - (deadband_vector.getY()) / (1 - deadband_vector.getY());

            next_translation = new Translation2d(
                    new_translation_x * DriveConstants.kMaxFloorSpeed,
                    new_translation_y * DriveConstants.kMaxFloorSpeed);

            return next_translation;
        }
    }

    public double getRotation() {
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation = (rightRotation - leftRotation) / 2.0;

        return Math.abs(combinedRotation) < 0.1 ? 0 : combinedRotation * DriveConstants.kMaxRotationSpeed; // TODO:
                                                                                                           // convert to
                                                                                                           // rad/s
    }

    public Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

}
