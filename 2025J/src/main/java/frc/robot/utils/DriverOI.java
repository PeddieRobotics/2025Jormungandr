package frc.robot.utils;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Superstructure.SuperstructureState;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.ReefCommands.AlignToHP;
import frc.robot.commands.ReefCommands.AlignToHPBasisVector;
import frc.robot.commands.ReefCommands.AlignToReef;
import frc.robot.commands.ReefCommands.AlignToReefBasisVector;
import frc.robot.commands.ReefCommands.OrbitReef;
// import frc.robot.commands.ReefCommands.AlignToReef2D;
// import frc.robot.commands.ReefCommands.AlignToReefEstimatedPose;
// import frc.robot.commands.ScoreCommands.AlignAndScore;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlign;
import frc.robot.utils.Constants.DriveConstants;

import static frc.robot.subsystems.Superstructure.SuperstructureState.HP_INTAKE;
import static frc.robot.subsystems.Superstructure.SuperstructureState.L1_PREP;
import static frc.robot.subsystems.Superstructure.SuperstructureState.L2_PREP;
import static frc.robot.subsystems.Superstructure.SuperstructureState.L3_PREP;
import static frc.robot.subsystems.Superstructure.SuperstructureState.L4_PREP;

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
        xButton.onTrue(new InstantCommand(() -> {
            if (Arrays.asList(SuperstructureState.STOW).contains(superstructure.getCurrentState()))
                superstructure.requestState(SuperstructureState.HP_INTAKE);
            else
                superstructure.requestState(SuperstructureState.STOW);
        }));

        Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        circleButton.onTrue(new InstantCommand(() -> superstructure.setL2Flag()));
        // circleButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.L2_PREP)));

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.onTrue(new InstantCommand(() -> superstructure.setL3Flag()));

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        // triangleButton.onTrue(new AlignAndScore(true)); //right align
        triangleButton.onTrue(new InstantCommand(() -> superstructure.setL4Flag()));
        // Claw.getInstance().stopClaw()));

        Trigger muteButton = new JoystickButton(controller, 15);
        // Set to climb
        muteButton.onTrue(new InstantCommand(() -> {
            Drivetrain.getInstance().resetTranslation(LimelightFrontMiddle.getInstance().getEstimatedPoseMT2().get().getTranslation());
        }));

        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.PRESTAGE)));
        // touchpadButton.onTrue(new SequentialCommandGroup(
        //     new InstantCommand(() -> {
        //         SmartDashboard.putBoolean("is inside bad hexagon", CalculateReefTarget.insideBadHexagon(Drivetrain.getInstance().getPose()));
        //     }),
        //     new InstantCommand(() -> {
        //         SmartDashboard.putNumber("align to tag", CalculateReefTarget.calculateTargetID());
        //     })
        // ));

        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        L1Bumper.onTrue(new InstantCommand(() -> superstructure.sendToScore()));

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        R1Bumper.whileTrue(new OrbitReef());

        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        Trigger L3Trigger = new JoystickButton(controller, PS4Controller.Button.kL3.value);
        L3Trigger.whileTrue(new ConditionalCommand(
            new AlignToReefBasisVector(AlignmentConstants.AlignmentDestination.LEFT, ReefAlign.kMaxSpeed, 0, 0),
            new AlignToReefBasisVector(AlignmentConstants.AlignmentDestination.MIDDLE, ReefAlign.kMaxSpeed, 0, 0),
            Claw.getInstance()::eitherCoralSensorTriggered
        ));

        Trigger R3Trigger = new JoystickButton(controller, PS4Controller.Button.kR3.value);
        R3Trigger.whileTrue(new ConditionalCommand(
            new AlignToReefBasisVector(AlignmentConstants.AlignmentDestination.RIGHT, ReefAlign.kMaxSpeed, 0, 0),
            new AlignToHPBasisVector(HPAlign.kMaxSpeed, HPAlign.kLateralOffset, HPAlign.kBackOffset, 0, 0),
            Claw.getInstance()::eitherCoralSensorTriggered
        ));

        Trigger optionButton = new JoystickButton(controller, PS4Controller.Button.kOptions.value);
        optionButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.EJECT_CORAL)));

        Trigger shareButton = new JoystickButton(controller, PS4Controller.Button.kShare.value);
        shareButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.EJECT_ALGAE)));
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
        combinedRotation = Math.signum(combinedRotation) * Math.pow(combinedRotation, 2);

        return Math.abs(combinedRotation) < 0.05 ? 0 : combinedRotation * DriveConstants.kMaxRotationSpeed; // TODO:
                                                                                                           // convert to
                                                                                                           // rad/s
    }

    public Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

}
