// TODO: rectify brokenness

package frc.robot.commands.ReefCommands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFrontLeft;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.CalculateReefTarget;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlignEstimatedPose;

public class AlignToHP extends Command {
    private Drivetrain drivetrain;
    private Limelight[] cameras;

    private PIDController translatePIDController, rotationPIDController;

    private double translateP, translateI, translateD, translateFF, translateThreshold;
    private double rotationP, rotationI, rotationD, rotationFF, rotationThreshold;
    private double rotationLowerP, rotationUseLowerPThreshold;
    private double maxSpeed;

    private Pose2d desiredPose;

    private String commandName;
    private double backMagnitude;
    private AlignmentDestination destination;

    private double xError, yError, rotationError;

    private final double stationAngle;

    public AlignToHP(AlignmentDestination destination, double maxSpeed, double backMagnitude) {
        drivetrain = Drivetrain.getInstance();
        
        cameras = new Limelight[] {
            LimelightFrontRight.getInstance(),
            LimelightFrontMiddle.getInstance(),
            LimelightFrontLeft.getInstance(),
        };

        translateP = ReefAlignEstimatedPose.kTranslateP;
        translateI = ReefAlignEstimatedPose.kTranslateI;
        translateD = ReefAlignEstimatedPose.kTranslateD;
        translateFF = ReefAlignEstimatedPose.kTranslateFF;
        translateThreshold = 0.02;

        rotationP = ReefAlignEstimatedPose.kRotationP;
        rotationI = ReefAlignEstimatedPose.kRotationI;
        rotationD = ReefAlignEstimatedPose.kRotationD;
        rotationFF = ReefAlignEstimatedPose.kRotationFF;
        rotationThreshold = ReefAlignEstimatedPose.kRotationThreshold;
        rotationLowerP = ReefAlignEstimatedPose.kRotationLowerP;
        rotationUseLowerPThreshold = ReefAlignEstimatedPose.kRotationUseLowerPThreshold;

        desiredPose = new Pose2d();
        stationAngle = 54.9;
        
        translatePIDController = new PIDController(translateP, translateI, translateD);
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        rotationPIDController.enableContinuousInput(-180.0, 180.0);
        
        this.destination = destination;
        this.maxSpeed = maxSpeed;
        this.backMagnitude = backMagnitude;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        switch (destination) {
            case LEFT:
                desiredPose = new Pose2d(1.417, 0.878, new Rotation2d(0));
                break;
            case MIDDLE:
                desiredPose = new Pose2d(0.87, 1.1, new Rotation2d(0));
                break;
            case RIGHT:
                desiredPose = new Pose2d(0.608, 1.456, new Rotation2d(0));
                break;
        }

        xError = 10000;
        yError = 10000;
        rotationError = 10000;

        LimelightFrontMiddle.getInstance().setLED(Limelight.LightMode.ON);
        SmartDashboard.putBoolean("HPAlign: Finished?", false);
    }
    
    private Pose2d getBestEstimatedPose() {
        for (Limelight camera : cameras) {
            Optional<Pose2d> measurement = camera.getEstimatedPoseMT2();
            if (measurement.isPresent() && camera.hasTarget())
                return measurement.get();
        }
        return drivetrain.getPose();
    }

    private boolean translationDistanceGood() {
        // sqrt(xError^2 + yError^2) < 0.04
        return xError * xError + yError * yError < Math.pow(translateThreshold, 2);
    }

    @Override
    public void execute() {
        if (DriverStation.isAutonomous())
            rotationError = drivetrain.getHeadingForceAdjust() - stationAngle;
        else
            rotationError = drivetrain.getHeading() - stationAngle;

        translatePIDController.setPID(translateP, translateI, translateD);
        if (Math.abs(rotationError) < rotationUseLowerPThreshold)
            rotationPIDController.setP(rotationLowerP);
        else
            rotationPIDController.setP(rotationP);
        rotationPIDController.setI(rotationI);
        rotationPIDController.setD(rotationD);

        double rotation = 0;
        if (Math.abs(rotationError) > rotationThreshold)
          rotation = rotationPIDController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

        Pose2d estimatedPose = getBestEstimatedPose();

        xError = estimatedPose.getX() - desiredPose.getX();
        yError = estimatedPose.getY() - desiredPose.getY();
        
        double xTranslate = 0, yTranslate = 0;
        if (!translationDistanceGood()) {
            xTranslate = translatePIDController.calculate(xError) + Math.signum(xError) * translateFF;
            yTranslate = translatePIDController.calculate(yError) + Math.signum(yError) * translateFF;
        }

        Translation2d translation = new Translation2d(xTranslate, yTranslate);
        double translateX = translation.getX();
        double translateY = translation.getY();
        double translateX_sgn = Math.signum(translateX);
        double translateY_sgn = Math.signum(translateY);
        double desaturatedX = Math.min(Math.abs(translateX), maxSpeed);
        double desaturatedY = Math.min(Math.abs(translateY), maxSpeed);
        translation = new Translation2d(translateX_sgn * desaturatedX, translateY_sgn * desaturatedY);

        drivetrain.driveForceAdjust(translation, rotation, true, null);
    }

    @Override
    public void end(boolean interrupted) {
        LimelightFrontMiddle.getInstance().setLED(Limelight.LightMode.OFF);
        SmartDashboard.putBoolean("HPAlign: Finished?", true);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rotationError) < rotationThreshold && translationDistanceGood();
    }
}
