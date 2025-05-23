package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToHPBasisVector;
import frc.robot.commands.AlignToReefBasisVector;
import frc.robot.commands.WaitForCoral;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlign;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Logger;

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

        // DISABLE FOR LAB!!!
        boolean isCompetition = true;

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
                ? stream.filter(auto -> auto.getName().startsWith("COMP"))
                : stream
        );

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
            // Robot pose supplier
            drivetrain::getPose,
            // Method to reset odometry (will be called if your auto has a starting pose)
            drivetrain::setPose,
            // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            drivetrain::getRobotRelativeSpeeds,
            // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
            drivetrain::driveRobotRelative,
            // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PPHolonomicDriveController(
                // Translation PID constants
                new PIDConstants(
                    AutoConstants.kTranslationP,
                    AutoConstants.kTranslationI,
                    AutoConstants.kTranslationD
                ),
                // Rotation PID constants
                new PIDConstants(
                    AutoConstants.kThetaP,
                    AutoConstants.kThetaI,
                    AutoConstants.kThetaD
                )
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
            // Reference to this subsystem to set requirements
            drivetrain
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

    public void registerReefAlignments(String namePrefix, double backOffset, double postScoreDelay, boolean isNotFirstPole, boolean isDaisy) {
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
                                blue, red, isNotFirstPole, isDaisy
                            ),
                            new WaitCommand(postScoreDelay),
                            new InstantCommand(() -> Logger.getInstance().logEvent("Auto Align to Reef converged", true))
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(2.0),
                            new InstantCommand(() -> superstructure.sendToScore()),
                            new WaitCommand(postScoreDelay),
                            new InstantCommand(() -> Logger.getInstance().logEvent("Auto Align to Reef timeout", true))
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
                                blue, red, isNotFirstPole, isDaisy
                            ),
                            new WaitCommand(postScoreDelay),
                            new InstantCommand(() -> Logger.getInstance().logEvent("Auto Align to Reef converged", true))
                        ),
                        new SequentialCommandGroup(
                            new WaitCommand(2.0),
                            new InstantCommand(() -> superstructure.sendToScore()),
                            new WaitCommand(postScoreDelay),
                            new InstantCommand(() -> Logger.getInstance().logEvent("Auto Align to Reef timeout", true))
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
        registerReefAlignments("", ReefAlign.kAutoTagBackMagnitude, 0.2, true, false);
        registerReefAlignments("CLOSE_", ReefAlign.kAutoCloseTagBackMagnitude, 0.3, false, false);

        registerReefAlignments("DAISY_", ReefAlign.kAutoTagBackMagnitude, 0.2, true, true);
        registerReefAlignments("DAISY_CLOSE_", ReefAlign.kAutoCloseTagBackMagnitude, 0.3, false, true);

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
                new AlignToHPBasisVector(HPAlign.kMaxSpeed, 0, HPAlign.kAutoBackOffset, 12, 2),
                // new WaitForCoral(),
                new WaitCommand(1.5)
            ),
            new InstantCommand(() -> {
                if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                    drivetrain.resetTranslation(new Translation2d(1.135, 1.042));
                else
                    drivetrain.resetTranslation(new Translation2d(16.415, 7.008));
            })
        ));
        NamedCommands.registerCommand("LEFT_1_ALIGN_TO_HP_12_2", new SequentialCommandGroup(
            new ParallelRaceGroup(
                new AlignToHPBasisVector(HPAlign.kMaxSpeed, 8 * 2.54 / 100, HPAlign.kAutoBackOffset, 12, 2),
                // new AlignToHPBasisVector(HPAlign.kMaxSpeed, 8 * 2.54 / 100 - 0.05, HPAlign.kAutoBackOffset, 12, 2),
                // new WaitForCoral(),
                new WaitCommand(1.5)
            ),
            new InstantCommand(() -> {
                if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                    drivetrain.resetTranslation(new Translation2d(1.299, 0.922));
                else
                    drivetrain.resetTranslation(new Translation2d(16.251, 7.128));
            })
        ));
        NamedCommands.registerCommand("RIGHT_1_ALIGN_TO_HP_13_1", new SequentialCommandGroup(
            new ParallelRaceGroup(
                new AlignToHPBasisVector(HPAlign.kMaxSpeed, -8 * 2.54 / 100, HPAlign.kAutoBackOffset, 13, 1),
                // new AlignToHPBasisVector(HPAlign.kMaxSpeed, -8 * 2.54 / 100 - 0.05, HPAlign.kAutoBackOffset, 13, 1),
                // new WaitForCoral(),
                new WaitCommand(1.5)
            ),
            new InstantCommand(() -> {
                if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                    drivetrain.resetTranslation(new Translation2d(1.299, 7.128));
                else
                    drivetrain.resetTranslation(new Translation2d(16.251, 0.922));
            })
        ));

        NamedCommands.registerCommand("ALIGN_TO_HP_13_1", new SequentialCommandGroup(
            new ParallelRaceGroup(
                new AlignToHPBasisVector(HPAlign.kMaxSpeed, 0, HPAlign.kAutoBackOffset, 13, 1),
                // new WaitForCoral(),
                new WaitCommand(1.5)
            ),
            new InstantCommand(() -> {
                if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                    drivetrain.resetTranslation(new Translation2d(1.135, 7.008));
                else
                    drivetrain.resetTranslation(new Translation2d(16.415, 1.042));
            })
        ));

        NamedCommands.registerCommand("WAIT_FOR_CORAL", new ParallelRaceGroup(
            new WaitForCoral(), new WaitCommand(0.5)
        ));

        NamedCommands.registerCommand("SET_ALGAE_REMOVAL", new InstantCommand(() -> {
            Superstructure.getInstance().setAutoRemoveAlgaeSwitch(true);
        }));

        NamedCommands.registerCommand("REMOVE_ALGAE_LEFT", new ParallelCommandGroup(
            new AlignToReefBasisVector(
                AlignmentConstants.AlignmentDestination.MIDDLE,
                ReefAlign.kMaxSpeed, 0, ReefAlign.kTagBackMagnitude,
                19, 6, true, false
            ),
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new InstantCommand(() -> {
                    boolean high = superstructure.isHighAlgae();
                    superstructure.requestState(high ? SuperstructureState.REEF2_ALGAE_INTAKE : SuperstructureState.REEF1_ALGAE_INTAKE);
                })
            )
        ));

        // NamedCommands.registerCommand("REMOVE_ALGAE_MIDDLE", new ParallelCommandGroup(
        //     new AlignToReefBasisVector(AlignmentConstants.AlignmentDestination.MIDDLE, ReefAlign.kMaxSpeed, 0, ReefAlign.kTagBackMagnitude, 18, 7, 0),
        //     new SequentialCommandGroup(
        //         new WaitCommand(0.25),
        //         new InstantCommand(() -> {
        //             boolean high = superstructure.isHighAlgae();
        //             superstructure.requestState(high ? SuperstructureState.REEF2_ALGAE_INTAKE : SuperstructureState.REEF1_ALGAE_INTAKE);
        //         })
        //     )
        // ));

        NamedCommands.registerCommand("REMOVE_ALGAE_RIGHT", new ParallelCommandGroup(
            new AlignToReefBasisVector(
                AlignmentConstants.AlignmentDestination.MIDDLE,
                ReefAlign.kMaxSpeed, 0, ReefAlign.kTagBackMagnitude,
                17, 8, true, false
            ),
            new SequentialCommandGroup(
                new WaitCommand(0.25),
                new InstantCommand(() -> {
                    boolean high = superstructure.isHighAlgae();
                    superstructure.requestState(high ? SuperstructureState.REEF2_ALGAE_INTAKE : SuperstructureState.REEF1_ALGAE_INTAKE);
                })
            )
        ));
    }
}
