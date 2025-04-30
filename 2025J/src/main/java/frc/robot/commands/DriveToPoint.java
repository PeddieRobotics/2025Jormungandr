package frc.robot.commands;

import com.ctre.phoenix6.jni.ErrorReportingJNI;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveToPoint extends Command {
    private Translation2d target;
    private double percentMovement;
    private Drivetrain drivetrain;

    private double errorMagnitude;
    private double translationThreshold;
    private ProfiledPIDController translationPIDController;

    public DriveToPoint(double x, double y, double percentMovement) {
        target = new Translation2d(x, y);
        percentMovement = this.percentMovement;
        drivetrain = Drivetrain.getInstance();

        translationPIDController = new ProfiledPIDController(
            0, 0, 0,
            new TrapezoidProfile.Constraints(3, 4)
        );
        translationPIDController.setGoal(0);
    }

    @Override
    public void initialize() {
        errorMagnitude = 10000;

        Translation2d current = drivetrain.getPose().getTranslation();
        double movement = target.minus(current).getNorm();
        translationThreshold = (1 - percentMovement) * movement + 0.02;
    }

    @Override
    public void execute() {
        Translation2d necessaryMovement = target.minus(drivetrain.getPose().getTranslation());
        errorMagnitude = necessaryMovement.getNorm();

        if (Math.abs(errorMagnitude) < translationThreshold)
            return;

        Translation2d unit = necessaryMovement.div(errorMagnitude);
        Translation2d translation = unit.times(translationPIDController.calculate(errorMagnitude));

        drivetrain.drive(translation, 0, true, null);
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