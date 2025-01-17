package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoCommands.*;

public class Autonomous extends SubsystemBase {

    private static SendableChooser<Command> autoChooser;

    private static Autonomous autonomous;
    private Drivetrain drivetrain;
    private RobotConfig config;

    public Autonomous() {
        drivetrain = Drivetrain.getInstance();

        configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    public void configureAutoBuilder() {
        AutoBuilder.configure(
                drivetrain::getPose, // Robot pose supplier
                drivetrain::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                drivetrain::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                drivetrain::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0, 0.0, 0.0) // Rotation PID constants
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

    public static Autonomous getInstance() {
        if (autonomous == null) {
            autonomous = new Autonomous();
        }
        return autonomous;
    }

    public static Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    public void registerNamedCommands(){
        NamedCommands.registerCommand("ALIGN_TO_REEF", new AlignToReefInAuto());
    }

}
