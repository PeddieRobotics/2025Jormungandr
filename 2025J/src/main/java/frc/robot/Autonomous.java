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
import frc.robot.commands.AlignToReefBasisVector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlign;
import frc.robot.utils.Logger;

public class Autonomous {
    private Drivetrain drivetrain;
    private Superstructure superstructure;
    private Logger logger;
    
    private Command createAlignToReef(AlignmentDestination destination, double backOffset, double postScoreDelay,
            int blueTag, int redTag, boolean isNotFirstPole, boolean isDaisy) {

        return new ParallelRaceGroup(
            new SequentialCommandGroup(
                new AlignToReefBasisVector(
                    destination,
                    ReefAlign.kMaxSpeed,
                    backOffset,
                    ReefAlign.kTagBackMagnitude,
                    blueTag, redTag,
                    isNotFirstPole, isDaisy
                ),
                new WaitCommand(postScoreDelay),
                new InstantCommand(() -> logger.logEvent("Auto Align to Reef converged", true))
            ),
            new SequentialCommandGroup(
                new WaitCommand(2.0),
                new InstantCommand(() -> superstructure.sendToScore()),
                new WaitCommand(postScoreDelay),
                new InstantCommand(() -> logger.logEvent("Auto Align to Reef timeout", true))
            )
        )
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
            false, false
        )
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
