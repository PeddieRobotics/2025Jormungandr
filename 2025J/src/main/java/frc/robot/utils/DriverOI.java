package frc.robot.utils;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Superstructure.SuperstructureState;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.AlignToBarge;
import frc.robot.commands.AlignToCage;
import frc.robot.commands.AlignToProcessor;
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
import frc.robot.subsystems.HPIntake;
import frc.robot.subsystems.LimelightFrontLeft;
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

    public enum DPadDirection {
        NONE, FORWARDS, LEFT, RIGHT, BACKWARDS
    };


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
        squareButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.ALGAE_GROUND_INTAKE)));

        Trigger triangleButton = new JoystickButton(controller, PS4Controller.Button.kTriangle.value);
        triangleButton.onTrue(new InstantCommand(() -> superstructure.sendToScore()));

        Trigger muteButton = new JoystickButton(controller, 15);
        muteButton.onTrue(new InstantCommand(() -> {
            Drivetrain.getInstance().resetTranslation(LimelightFrontLeft.getInstance().getEstimatedPoseMT2().get().getTranslation());
            // Drivetrain.getInstance().resetTranslation(LimelightFrontMiddle.getInstance().getEstimatedPoseMT2().get().getTranslation());
        }));

        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.PRESTAGE)));

        // TODO: Set binding to enter climb mode
        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        L1Bumper.onTrue(new InstantCommand(() -> HPIntake.getInstance().extendLinearActuator()));

        // TODO: Set binding to climb
        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        // R1Bumper.onTrue(new InstantCommand(() -> HPIntake.getInstance().retractLinearActuator()));
        R1Bumper.whileTrue(new AlignToCage());

        // DO NOT BIND: USED FOR ROTATION OF DRIVETRAIN
        Trigger L2Trigger = new JoystickButton(controller, PS4Controller.Button.kL2.value);

        // DO NOT BIND: USED FOR ROTATION OF DRIVETRAIN
        Trigger R2Trigger = new JoystickButton(controller, PS4Controller.Button.kR2.value);

        Trigger L3Trigger = new JoystickButton(controller, PS4Controller.Button.kL3.value);
        L3Trigger.whileTrue(new ConditionalCommand(
            new AlignToBarge(), 
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new AlignToReefBasisVector(AlignmentConstants.AlignmentDestination.LEFT, ReefAlign.kMaxSpeed, 0, 0),
                    new ConditionalCommand(
                        new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new ParallelCommandGroup(
                                new AlignToReefBasisVector(AlignmentConstants.AlignmentDestination.MIDDLE, ReefAlign.kMaxSpeed, 0, 0),
                                new SequentialCommandGroup(
                                    new WaitCommand(0.25),
                                    new InstantCommand(() -> {
                                        Optional<Boolean> high = superstructure.isHighAlgae();
                                        if (high.isEmpty() || high.get())
                                            superstructure.requestState(SuperstructureState.REEF2_ALGAE_INTAKE);
                                        else
                                            superstructure.requestState(SuperstructureState.REEF1_ALGAE_INTAKE);
                                    })
                                )
                            )
                        ), 
                        new InstantCommand(), 
                        OperatorOI.getInstance()::getLeftBumperHeld
                    )
                ),
                new AlignToReefBasisVector(AlignmentConstants.AlignmentDestination.MIDDLE, ReefAlign.kMaxSpeed, 0, 0), 
                Claw.getInstance()::eitherCoralSensorTriggered
            ),
            Claw.getInstance()::getAlgaeSensor
        ));

        Trigger R3Trigger = new JoystickButton(controller, PS4Controller.Button.kR3.value);
        R3Trigger.whileTrue(new ConditionalCommand(
            new AlignToProcessor(2),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new AlignToReefBasisVector(AlignmentConstants.AlignmentDestination.RIGHT, ReefAlign.kMaxSpeed, 0, 0),
                    new ConditionalCommand(
                        new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new ParallelCommandGroup(
                                new AlignToReefBasisVector(AlignmentConstants.AlignmentDestination.MIDDLE, ReefAlign.kMaxSpeed, 0, 0),
                                new SequentialCommandGroup(
                                    new WaitCommand(0.25), 
                                    new InstantCommand(() -> {
                                        Optional<Boolean> high = superstructure.isHighAlgae();
                                        if (high.isEmpty() || high.get())
                                            superstructure.requestState(SuperstructureState.REEF2_ALGAE_INTAKE);
                                        else
                                            superstructure.requestState(SuperstructureState.REEF1_ALGAE_INTAKE);
                                    })
                                )
                            )
                        ),
                        new InstantCommand(),
                        OperatorOI.getInstance()::getLeftBumperHeld
                    )
                ),
                new AlignToHPBasisVector(HPAlign.kMaxSpeed, HPAlign.kLateralOffset, HPAlign.kBackOffset, 0, 0),
                Claw.getInstance()::eitherCoralSensorTriggered
            ),
            Claw.getInstance()::getAlgaeSensor
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

    public DPadDirection getDriverDPadInput() {
        switch (controller.getPOV()) {
            case 0:
                return DPadDirection.FORWARDS;
            case 90:
                return DPadDirection.RIGHT;
            case 270:
                return DPadDirection.LEFT;
            case 180:
                return DPadDirection.BACKWARDS;
            default:
                return DPadDirection.NONE;
        }
    }

    public Translation2d getCardinalDirection() {
        switch (getDriverDPadInput()) {
            case FORWARDS:
                return new Translation2d(DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed,
                        0.0);
            case RIGHT:
                return new Translation2d(0.0,
                        -DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed);
            case LEFT:
                return new Translation2d(0.0,
                        DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed);
            case BACKWARDS:
                return new Translation2d(-DriveConstants.kCardinalDirectionSpeedScale * DriveConstants.kMaxFloorSpeed,
                        0.0);
            default:
                return new Translation2d(0.0, 0.0);
        }

    }

}
