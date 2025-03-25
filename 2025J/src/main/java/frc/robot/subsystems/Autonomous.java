package frc.robot.subsystems;

import static frc.robot.subsystems.Superstructure.SuperstructureState.HP_INTAKE;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCommands.*;
import frc.robot.commands.ReefCommands.AlignToHP;
import frc.robot.commands.ReefCommands.AlignToHPBasisVector;
import frc.robot.commands.ReefCommands.AlignToReef;
import frc.robot.commands.ReefCommands.AlignToReefBasisVector;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlign;
import frc.robot.utils.Constants.AutoConstants;

public class Autonomous extends SubsystemBase {

    private static SendableChooser<Command> autoChooser;

    private static Autonomous autonomous;
    private Drivetrain drivetrain;
    private RobotConfig config;
    private Superstructure superstructure;

    public Autonomous() {
        drivetrain = Drivetrain.getInstance();

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        registerNamedCommands();
        configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoSelector", autoChooser);
        superstructure = Superstructure.getInstance();
    }

    public static Autonomous getInstance() {
        if (autonomous == null) {
            autonomous = new Autonomous();
        }
        return autonomous;
    }

    public void configureAutoBuilder() {
        AutoBuilder.configure(
            drivetrain::getPose, // Robot pose supplier
            drivetrain::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            drivetrain::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                            // holonomic drive trains
                    // TODO: update these constants for AUTO
                    new PIDConstants(AutoConstants.kTranslationP, AutoConstants.kTranslationI,
                            AutoConstants.kTranslationD), // Translation PID constants
                    new PIDConstants(AutoConstants.kThetaP, AutoConstants.kThetaI, AutoConstants.kThetaD) // Rotation
                                                                                                            // PID
                                                                                                            // constants
            ),
            config,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE RED SIDE (FOR THE LAB)

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return true;
            },
            drivetrain // Reference to this subsystem to set requirements
        );
    }

    public static Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    private static final Map<Integer, Translation2d> kReefScoringLocationLeft = new HashMap<>() {{
        put(17, new Translation2d(3.672, 3.007));
        put(18, new Translation2d(3.198, 4.225));
        put(19, new Translation2d(4.016, 5.243));
        put(20, new Translation2d(5.307, 5.044));
        put(21, new Translation2d(5.781, 3.827));
        put(22, new Translation2d(4.962, 2.808));
    }};
    private static final Map<Integer, Translation2d> kReefScoringLocationRight = new HashMap<>() {{
        put(17, new Translation2d(3.961, 2.840));
        put(18, new Translation2d(3.198, 3.891));
        put(19, new Translation2d(3.727, 5.076));
        put(20, new Translation2d(5.018, 5.211));
        put(21, new Translation2d(5.781, 4.161));
        put(22, new Translation2d(5.252, 2.975));
    }};

    public void registerReefAlignments(String namePrefix, double backOffset, double postScoreDelay) {
        for (int blue = 17; blue <= 22; blue++) {
            int red = AlignmentConstants.kBlueToRedReefTag.get(blue);

            Translation2d poseBlueLeft = kReefScoringLocationLeft.get(blue);
            double poseBlueLeftX = poseBlueLeft.getX();
            double poseBlueLeftY = poseBlueLeft.getY();
            double poseRedLeftX = 17.55 - poseBlueLeftX;
            double poseRedLeftY = 8.05 - poseBlueLeftY;

            Translation2d poseBlueRight = kReefScoringLocationRight.get(blue);
            double poseBlueRightX = poseBlueRight.getX();
            double poseBlueRightY = poseBlueRight.getY();
            double poseRedRightX = 17.55 - poseBlueRightX;
            double poseRedRightY = 8.05 - poseBlueRightY;

            NamedCommands.registerCommand(
                namePrefix + "ALIGN_" + blue + "_" + red + "_LEFT",
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            new AlignToReefBasisVector(
                                Constants.AlignmentConstants.AlignmentDestination.LEFT,
                                ReefAlign.kMaxSpeed, backOffset, ReefAlign.kTagBackMagnitude,
                                blue, red
                            ),
                            new WaitCommand(postScoreDelay)
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(2.0),
                            new InstantCommand(() -> superstructure.sendToScore()),
                            new WaitCommand(postScoreDelay)
                        )
                    ),
                    new InstantCommand(() -> {
                        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                            drivetrain.resetTranslation(new Translation2d(poseBlueLeftX, poseBlueLeftY));
                        else
                            drivetrain.resetTranslation(new Translation2d(poseRedLeftX, poseRedLeftY));
                    })        
                )
            );
            NamedCommands.registerCommand(
                namePrefix + "ALIGN_" + blue + "_" + red + "_RIGHT",
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new SequentialCommandGroup(
                            new AlignToReefBasisVector(
                                Constants.AlignmentConstants.AlignmentDestination.RIGHT,
                                ReefAlign.kMaxSpeed, backOffset, ReefAlign.kTagBackMagnitude,
                                blue, red
                            ),
                            new WaitCommand(postScoreDelay)
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(2.0),
                            new InstantCommand(() -> superstructure.sendToScore()),
                            new WaitCommand(postScoreDelay)
                        )
                    ),
                    new InstantCommand(() -> {
                        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                            drivetrain.resetTranslation(new Translation2d(poseBlueRightX, poseBlueRightY));
                        else
                            drivetrain.resetTranslation(new Translation2d(poseRedRightX, poseRedRightY));
                    }) 
                )
            );
        }
    }
        
    private void registerNamedCommands() {
        registerReefAlignments("", ReefAlign.kAutoTagBackMagnitude + 0.01, 0.2);
        registerReefAlignments("CLOSE_", ReefAlign.kAutoTagBackMagnitude - 0.01, 0.3);

        NamedCommands.registerCommand("L1_PREP", new InstantCommand(() -> superstructure.requestState(SuperstructureState.L1_PREP)));
        NamedCommands.registerCommand("L2_PREP", new InstantCommand(() -> superstructure.requestState(SuperstructureState.L2_PREP)));
        NamedCommands.registerCommand("L3_PREP", new InstantCommand(() -> superstructure.requestState(SuperstructureState.L3_PREP)));
        NamedCommands.registerCommand("PRESTAGE", new InstantCommand(() -> superstructure.requestState(SuperstructureState.PRESTAGE)));
        NamedCommands.registerCommand("L4_PREP", new InstantCommand(() -> superstructure.requestState(SuperstructureState.L4_PREP)));
        NamedCommands.registerCommand("STOW", new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));
        NamedCommands.registerCommand("HP_INTAKE", new InstantCommand(() -> superstructure.requestState(SuperstructureState.HP_INTAKE)));
        NamedCommands.registerCommand("SEND_TO_SCORE", new InstantCommand(() -> superstructure.sendToScore()));

        NamedCommands.registerCommand("ALIGN_TO_HP_12_2", new SequentialCommandGroup(
            new ParallelRaceGroup(
                new AlignToHPBasisVector(HPAlign.kMaxSpeed, HPAlign.kAutoRightLateralOffset, HPAlign.kAutoBackOffset, 12, 2),
                new WaitForCoral(), new WaitCommand(1.5)
            ),
            new InstantCommand(() -> {
                if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                    drivetrain.resetTranslation(new Translation2d(1.098, 0.995));
                else
                    drivetrain.resetTranslation(new Translation2d(16.452, 7.055));
            })
        ));
        NamedCommands.registerCommand("ALIGN_TO_HP_13_1", new SequentialCommandGroup(
            new ParallelRaceGroup(
                new AlignToHPBasisVector(HPAlign.kMaxSpeed, HPAlign.kAutoLeftLateralOffset, HPAlign.kAutoBackOffset, 13, 1),
                new WaitForCoral(), new WaitCommand(1.5)
            ),
            new InstantCommand(() -> {
                if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                    drivetrain.resetTranslation(new Translation2d(1.098, 7.055));
                else
                    drivetrain.resetTranslation(new Translation2d(16.452, 0.995));
            })
        ));

        NamedCommands.registerCommand("WAIT_FOR_CORAL", new ParallelRaceGroup(
            new WaitForCoral(), new WaitCommand(0.5)
        ));

        NamedCommands.registerCommand("SET_ALGAE_REMOVAL", new InstantCommand(() -> {
            Superstructure.getInstance().setAutoRemoveAlgaeSwitch(true);
        }));
    }
}
