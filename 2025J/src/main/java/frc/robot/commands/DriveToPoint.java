package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveToPoint extends Command {
    private Drivetrain drivetrain;

    private Translation2d targetRelativeToOwnSide, targetBlue;
    private double percentMovement;
    private double errorMagnitude, translationThreshold;
    private ProfiledPIDController translationPIDController, rotationPIDController;

    public DriveToPoint(double x, double y, double rotationTarget, double percentMovement) {
        targetRelativeToOwnSide = new Translation2d(x, y);
        this.percentMovement = percentMovement;
        drivetrain = Drivetrain.getInstance();

        translationPIDController = new ProfiledPIDController(
            4, 0, 0,
            new TrapezoidProfile.Constraints(3, 4)
        );
        translationPIDController.setGoal(0);

        rotationPIDController = new ProfiledPIDController(
            0.08, 0, 0,
            new TrapezoidProfile.Constraints(540, 720)
        );
        rotationPIDController.enableContinuousInput(-180, 180);
        rotationPIDController.setGoal(rotationTarget);

        // SmartDashboard.putNumber("DriveToPoint Translation P", 5);
        // SmartDashboard.putNumber("DriveToPoint Rotation P", 0.08);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            targetBlue = targetRelativeToOwnSide;
        else
            targetBlue = new Translation2d(17.55, 8.05).minus(targetRelativeToOwnSide);

        errorMagnitude = 10000;

        Translation2d current = drivetrain.getPose().getTranslation();
        double movement = targetBlue.minus(current).getNorm();
        translationThreshold = (1.0 - percentMovement) * movement + 0.02;

        SmartDashboard.putNumber("DriveToPoint target x", targetBlue.getX());
        SmartDashboard.putNumber("DriveToPoint target y", targetBlue.getY());
        SmartDashboard.putNumber("DriveToPoint current x", current.getX());
        SmartDashboard.putNumber("DriveToPoint current y", current.getY());
        SmartDashboard.putNumber("DriveToPoint movement", movement);
        SmartDashboard.putNumber("DriveToPoint threshold", translationThreshold);
    }

    @Override
    public void execute() {
        // translationPIDController.setP(SmartDashboard.getNumber("DriveToPoint Translation P", 0));
        // rotationPIDController.setP(SmartDashboard.getNumber("DriveToPoint Rotation P", 0));
        
        Translation2d necessaryMovement = drivetrain.getPose().getTranslation().minus(targetBlue);
        errorMagnitude = necessaryMovement.getNorm();

        if (Math.abs(errorMagnitude) < translationThreshold)
            return;

        Translation2d unit = necessaryMovement.div(errorMagnitude);
        Translation2d translation = unit.times(translationPIDController.calculate(errorMagnitude));
        
        double rotationError = drivetrain.getHeading();
        double rotation = 0;
        if (Math.abs(rotationError) >= 0.5)
            rotation = rotationPIDController.calculate(rotationError);

        drivetrain.driveBlue(translation, rotation, true, null);
    }
    

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(errorMagnitude) < translationThreshold;
    }
}