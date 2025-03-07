package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCommands.*;
import frc.robot.commands.ReefCommands.AlignToHP;
import frc.robot.commands.ReefCommands.AlignToReef;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;
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

    public void registerNamedCommands(){
        for (int blue = 17; blue <= 22; blue++) {
            int red = AlignmentConstants.kBlueToRedReefTag.get(blue);
            NamedCommands.registerCommand(
                "ALIGN_" + blue + "_" + red + "_LEFT",
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                        new AlignToReef(Constants.AlignmentConstants.AlignmentDestination.LEFT, 2.0, blue, red),
                        new WaitCommand(0.3)
                    ),
                    new WaitCommand(2)
                )
            );
            NamedCommands.registerCommand(
                "ALIGN_" + blue + "_" + red + "_RIGHT",
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                        new AlignToReef(Constants.AlignmentConstants.AlignmentDestination.RIGHT, 2.0, blue, red),
                        new WaitCommand(0.3)
                    ),
                    new WaitCommand(2)
                )
            );
        }

        NamedCommands.registerCommand("L1_PREP", new InstantCommand(() -> superstructure.requestState(SuperstructureState.L1_PREP)));
        NamedCommands.registerCommand("L2_PREP", new InstantCommand(() -> superstructure.requestState(SuperstructureState.L2_PREP)));
        NamedCommands.registerCommand("L3_PREP", new InstantCommand(() -> superstructure.requestState(SuperstructureState.L3_PREP)));
        NamedCommands.registerCommand("L3L4_PRESTAGE", new InstantCommand(() -> superstructure.requestState(SuperstructureState.L3L4_PRESTAGE)));
        NamedCommands.registerCommand("L4_PREP", new InstantCommand(() -> superstructure.requestState(SuperstructureState.L4_PREP)));
        NamedCommands.registerCommand("STOW", new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));
        NamedCommands.registerCommand("HP_INTAKE", new InstantCommand(() -> superstructure.requestState(SuperstructureState.HP_INTAKE)));
        NamedCommands.registerCommand("SEND_TO_SCORE", new InstantCommand(() -> superstructure.sendToScore()));

        NamedCommands.registerCommand("ALIGN_TO_HP", new ParallelRaceGroup(
            new AlignToHP(HPAlign.kMaxSpeed, HPAlign.kLateralOffset, HPAlign.kBackOffset),
            new WaitForCoral(), new WaitCommand(2)
        ));
        NamedCommands.registerCommand("WAIT_FOR_CORAL", new ParallelRaceGroup(
            new WaitForCoral(), new WaitCommand(2)
        ));

        NamedCommands.registerCommand("SET_ALGAE_REMOVAL", new InstantCommand(() -> {
            Superstructure.getInstance().setAutoRemoveAlgaeSwitch(true);
        }));
    }
}
