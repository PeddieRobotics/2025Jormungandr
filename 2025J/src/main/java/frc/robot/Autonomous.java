package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlignToHPBasisVector;
import frc.robot.commands.AlignToReefBasisVector;
import frc.robot.commands.DriveToPoint;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlign;
import frc.robot.utils.Logger;

public class Autonomous {
    private Drivetrain drivetrain;
    private Superstructure superstructure;
    private Logger logger;

    private Command createRightAlignToHP() {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new AlignToHPBasisVector(HPAlign.kMaxSpeed, 8 * 2.54 / 100, HPAlign.kAutoBackOffset, 12, 2),
                // new AlignToHPBasisVector(HPAlign.kMaxSpeed, 8 * 2.54 / 100 - 0.05, HPAlign.kAutoBackOffset, 12, 2),
                // new WaitForCoral(),
                new WaitCommand(3.0)
            ),
            new InstantCommand(() -> {
                if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                    drivetrain.resetTranslation(new Translation2d(1.299, 0.922));
                else
                    drivetrain.resetTranslation(new Translation2d(16.251, 7.128));
            })
        );
    }
    private Command createLeftAlignToHP() {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new AlignToHPBasisVector(HPAlign.kMaxSpeed, -8 * 2.54 / 100, HPAlign.kAutoBackOffset, 13, 1),
                // new AlignToHPBasisVector(HPAlign.kMaxSpeed, -8 * 2.54 / 100 - 0.05, HPAlign.kAutoBackOffset, 13, 1),
                // new WaitForCoral(),
                new WaitCommand(3.0)
            ),
            new InstantCommand(() -> {
                if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                    drivetrain.resetTranslation(new Translation2d(1.299, 7.128));
                else
                    drivetrain.resetTranslation(new Translation2d(16.251, 0.922));
            })
        );
    }

    private Command createAlignToReef(AlignmentDestination destination, double backOffset, double postScoreDelay,
            int blueTag, int redTag, boolean isFirstPole, boolean isDaisy) {

        return new ParallelRaceGroup(
            new SequentialCommandGroup(
                new AlignToReefBasisVector(
                    destination,
                    ReefAlign.kMaxSpeed,
                    backOffset,
                    ReefAlign.kTagBackMagnitude,
                    blueTag, redTag,
                    !isFirstPole, isDaisy
                ),
                new WaitCommand(postScoreDelay),
                new InstantCommand(() -> logger.logEvent("Auto Align to Reef converged", true))
            ),
            new SequentialCommandGroup(
                new WaitCommand(4.0),
                new InstantCommand(() -> superstructure.sendToScore()),
                new WaitCommand(postScoreDelay),
                new InstantCommand(() -> logger.logEvent("Auto Align to Reef timeout", true))
            )
        );
    }
    
    public final Command right4pieceAuto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            drivetrain.setStartingPose(new Translation2d(7.050, 2.686));
            superstructure.requestState(SuperstructureState.PRESTAGE);
        }),
        createAlignToReef(
            AlignmentDestination.RIGHT,
            ReefAlign.kAutoCloseTagBackMagnitude,
            0.3, 22, 9,
            true, false
        ),
        new DriveToPoint(3.305, 1.5, 120.0, 0.5),
        createRightAlignToHP(),
        createAlignToReef(
            AlignmentDestination.LEFT,
            ReefAlign.kAutoCloseTagBackMagnitude,
            0.2, 17, 8,
            false, false
        ),
        createRightAlignToHP(),
        createAlignToReef(
            AlignmentDestination.RIGHT,
            ReefAlign.kAutoCloseTagBackMagnitude,
            0.2, 17, 8,
            false, false
        ),
        createRightAlignToHP(),
        createAlignToReef(
            AlignmentDestination.RIGHT,
            ReefAlign.kAutoCloseTagBackMagnitude,
            0.2, 18, 7,
            false, false
        )
    );

    private final Command right1pieceAuto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            drivetrain.setStartingPose(new Translation2d(7.050, 2.686));
            superstructure.requestState(SuperstructureState.PRESTAGE);
        }),         
        createAlignToReef(
            AlignmentDestination.LEFT,
            ReefAlign.kAutoCloseTagBackMagnitude, 
            0.3,  22, 9, 
            true, false
        )
    );

    private final Command driveAuto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            drivetrain.setStartingPose(new Translation2d(5.0, 3.0));
        }),
        new DriveToPoint(4.0, 3.0, 120.0, 1.0)
    );

    public final Command waitAuto = new WaitCommand(1);
    
    private static Autonomous autonomous;
    public static Autonomous getInstance() {
        if (autonomous == null)
            autonomous = new Autonomous();
        return autonomous;
    }

    private SendableChooser<Command> autoChooser;
    private SendableChooser<Double> autoStartPosition;

    public Autonomous() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Right 4 Piece", right4pieceAuto);
        autoChooser.setDefaultOption("Right 1 Piece", right1pieceAuto);
        autoChooser.setDefaultOption("1 meter", driveAuto);
        autoChooser.addOption("Wait", waitAuto);
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        autoStartPosition = new SendableChooser<>();
        autoStartPosition.setDefaultOption("NONE/TELEOP", 0.0);
        autoStartPosition.addOption("LEFT", -90.0);
        autoStartPosition.addOption("RIGHT", 90.0);
        autoStartPosition.addOption("CENTER", 180.0);
        autoStartPosition.addOption("RIGHT JIG", 120.0);
        autoStartPosition.addOption("LEFT JIG", -120.0);
        SmartDashboard.putData("Auto Starting Direction", autoStartPosition);

        drivetrain = Drivetrain.getInstance();
        superstructure = Superstructure.getInstance();
        logger = Logger.getInstance();
    }

    public double getStartHeading() {
        return autoStartPosition.getSelected();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
