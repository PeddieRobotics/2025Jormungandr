package frc.robot.commands.ReefCommands;

import java.util.Optional;

import org.ejml.dense.block.TriangularSolver_MT_DDRB;

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
import frc.robot.subsystems.LimelightBack;
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
    private double desiredAngle;

    private double backOffset, lateralOffset;

    private double xError, yError, rotationError;

    public AlignToHP(double maxSpeed, double lateralOffset, double backOffset) {
        drivetrain = Drivetrain.getInstance();
        
        cameras = new Limelight[] {
            LimelightBack.getInstance(),
            LimelightFrontLeft.getInstance(),
            LimelightFrontRight.getInstance(),
            LimelightFrontMiddle.getInstance(),
        };

        translateP = ReefAlignEstimatedPose.kTranslateP;
        translateI = ReefAlignEstimatedPose.kTranslateI;
        translateD = ReefAlignEstimatedPose.kTranslateD;
        translateFF = ReefAlignEstimatedPose.kTranslateFF;
        translateThreshold = ReefAlignEstimatedPose.kTranslateDistanceThreshold;

        rotationP = ReefAlignEstimatedPose.kRotationP;
        rotationI = ReefAlignEstimatedPose.kRotationI;
        rotationD = ReefAlignEstimatedPose.kRotationD;
        rotationFF = ReefAlignEstimatedPose.kRotationFF;
        rotationThreshold = ReefAlignEstimatedPose.kRotationThreshold;
        rotationLowerP = ReefAlignEstimatedPose.kRotationLowerP;
        rotationUseLowerPThreshold = ReefAlignEstimatedPose.kRotationUseLowerPThreshold;
        
        translatePIDController = new PIDController(translateP, translateI, translateD);
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        rotationPIDController.enableContinuousInput(-180.0, 180.0);
        
        this.maxSpeed = maxSpeed;
        this.lateralOffset = lateralOffset;
        this.backOffset = backOffset;

        SmartDashboard.putNumber("HP Align: lateral offset", lateralOffset);
        SmartDashboard.putNumber("HP Align: back offset", backOffset);

        addRequirements(drivetrain);
        
        {
            SmartDashboard.putNumber("HP Align: translateP", translateP);
            SmartDashboard.putNumber("HP Align: translateI", translateI);
            SmartDashboard.putNumber("HP Align: translateD", translateD);
            SmartDashboard.putNumber("HP Align: translateFF", translateFF);
            SmartDashboard.putNumber("HP Align: translateThreshold", translateThreshold);
            
            SmartDashboard.putNumber("HP Align: rotationP", rotationP);
            SmartDashboard.putNumber("HP Align: rotationI", rotationI);
            SmartDashboard.putNumber("HP Align: rotationD", rotationD);
            SmartDashboard.putNumber("HP Align: rotationFF", rotationFF);
            SmartDashboard.putNumber("HP Align: rotationThreshold", rotationThreshold);
            SmartDashboard.putNumber("HP Align: rotationLowerP", rotationLowerP);
            SmartDashboard.putNumber("HP Align: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
        }
    }

    @Override
    public void initialize() {
        // blue: 12, 13
        // red: 1, 2
        Pose2d tagPose = Limelight.getAprilTagPose(12);
        double tagAngle = tagPose.getRotation().getRadians();

        lateralOffset = SmartDashboard.getNumber("HP Align: lateral offset", lateralOffset);
        backOffset = SmartDashboard.getNumber("HP Align: back offset", backOffset);

        desiredPose = new Pose2d(
            tagPose.getX() + backOffset * Math.cos(tagAngle) + lateralOffset * Math.sin(tagAngle),
            tagPose.getY() + backOffset * Math.sin(tagAngle) - lateralOffset * Math.cos(tagAngle),
            new Rotation2d(0)
        );

        xError = 10000;
        yError = 10000;
        rotationError = 10000;

        LimelightFrontMiddle.getInstance().setLED(Limelight.LightMode.ON);
        SmartDashboard.putBoolean("HP Align: Finished?", false);
        
        {
            // TODO: use these values to determine what the path starting waypoint should be
            String thing = "HPAlign: (" + lateralOffset + ", " + backOffset + ") ";
            SmartDashboard.putNumber(thing + "x", desiredPose.getX());
            SmartDashboard.putNumber(thing + "y", desiredPose.getY());
        }
    }
    
    private Optional<Pose2d> getBestEstimatedPose() {
        for (Limelight camera : cameras) {
            Optional<Pose2d> measurement = camera.getEstimatedPoseMT2();
            if (measurement.isPresent() && camera.hasTarget())
                return Optional.of(measurement.get());
        }

        return Optional.empty();
    }

    private boolean translationDistanceGood() {
        // sqrt(xError^2 + yError^2) < 0.04
        return xError * xError + yError * yError < Math.pow(translateThreshold, 2);
    }

    @Override
    public void execute() {
        {
            translateP = SmartDashboard.getNumber("HP Align: translateP", translateP);
            translateI = SmartDashboard.getNumber("HP Align: translateI", translateI);
            translateD = SmartDashboard.getNumber("HP Align: translateD", translateD);
            translateFF = SmartDashboard.getNumber("HP Align: translateFF", translateFF);
            translateThreshold = SmartDashboard.getNumber("HP Align: translateThreshold", translateThreshold);

            rotationP = SmartDashboard.getNumber("HP Align: rotationP", rotationP);
            rotationI = SmartDashboard.getNumber("HP Align: rotationI", rotationI);
            rotationD = SmartDashboard.getNumber("HP Align: rotationD", rotationD);
            rotationFF = SmartDashboard.getNumber("HP Align: rotationFF", rotationFF);
            rotationThreshold = SmartDashboard.getNumber("HP Align: rotationThreshold", rotationThreshold);
            rotationLowerP = SmartDashboard.getNumber("HP Align: rotationLowerP", rotationLowerP);
            rotationUseLowerPThreshold = SmartDashboard.getNumber("HP Align: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
        }
        
        if (DriverStation.isAutonomous())
            rotationError = drivetrain.getHeadingForceAdjust() - desiredAngle;
        else
            rotationError = drivetrain.getHeading() - desiredAngle;

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

        Optional<Pose2d> estimatedPoseOptional = getBestEstimatedPose();
        if (!estimatedPoseOptional.isPresent()) {
            drivetrain.drive(new Translation2d(0, 0), 0, true, null);
            return;
        }
        Pose2d estimatedPose = estimatedPoseOptional.get();

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

        {
            SmartDashboard.putNumber("HP Align: xError", xError);
            SmartDashboard.putNumber("HP Align: yError", yError);
            SmartDashboard.putNumber("HP Align: rotationError", rotationError);

            SmartDashboard.putBoolean("HP Align: translation good?", translationDistanceGood());
            SmartDashboard.putBoolean("HP Align: rotation good?", Math.abs(rotationError) < rotationThreshold);
        }
    }

    @Override
    public void end(boolean interrupted) {
        LimelightFrontMiddle.getInstance().setLED(Limelight.LightMode.OFF);
        SmartDashboard.putBoolean("HP Align: Finished?", true);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rotationError) < rotationThreshold && translationDistanceGood();
    }
}
