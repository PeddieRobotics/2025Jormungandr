package frc.robot.commands.ReefCommands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFrontLeft;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.utils.CalculateHPTarget;
import frc.robot.utils.CalculateReefTarget;
import frc.robot.utils.Logger;
import frc.robot.utils.MagnitudeCap;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;

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

    private int blueTargetTag, redTargetTag;

    public AlignToHP(double maxSpeed, double lateralOffset, double backOffset, int blueTargetTag, int redTargetTag) {
        drivetrain = Drivetrain.getInstance();
        
        cameras = new Limelight[] {
            LimelightBack.getInstance(),
            LimelightFrontLeft.getInstance(),
            LimelightFrontRight.getInstance(),
            LimelightFrontMiddle.getInstance(),
        };

        translateP = HPAlign.kTranslateP;
        translateI = HPAlign.kTranslateI;
        translateD = HPAlign.kTranslateD;
        translateFF = HPAlign.kTranslateFF;
        translateThreshold = HPAlign.kTranslateThreshold;

        rotationP = HPAlign.kRotationP;
        rotationI = HPAlign.kRotationI;
        rotationD = HPAlign.kRotationD;
        rotationFF = HPAlign.kRotationFF;
        rotationThreshold = HPAlign.kRotationThreshold;
        rotationLowerP = HPAlign.kRotationLowerP;
        rotationUseLowerPThreshold = HPAlign.kRotationUseLowerPThreshold;
        
        translatePIDController = new PIDController(translateP, translateI, translateD);
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        rotationPIDController.enableContinuousInput(-180.0, 180.0);
        
        this.maxSpeed = maxSpeed;
        this.lateralOffset = lateralOffset;
        this.backOffset = backOffset;
        this.blueTargetTag = blueTargetTag;
        this.redTargetTag = redTargetTag;

        SmartDashboard.putNumber("HPAlign: lateral offset", lateralOffset);
        SmartDashboard.putNumber("HPAlign: back offset", backOffset);
        // SmartDashboard.putNumber("HPAlign: max speed", maxSpeed);

        addRequirements(drivetrain);
        
        // {
        //     SmartDashboard.putNumber("HPAlign: translateP", translateP);
        //     SmartDashboard.putNumber("HPAlign: translateI", translateI);
        //     SmartDashboard.putNumber("HPAlign: translateD", translateD);
        //     SmartDashboard.putNumber("HPAlign: translateFF", translateFF);
            SmartDashboard.putNumber("HPAlign: translateThreshold", translateThreshold);
            
        //     SmartDashboard.putNumber("HPAlign: rotationP", rotationP);
        //     SmartDashboard.putNumber("HPAlign: rotationI", rotationI);
        //     SmartDashboard.putNumber("HPAlign: rotationD", rotationD);
        //     SmartDashboard.putNumber("HPAlign: rotationFF", rotationFF);
        //     SmartDashboard.putNumber("HPAlign: rotationThreshold", rotationThreshold);
        //     SmartDashboard.putNumber("HPAlign: rotationLowerP", rotationLowerP);
        //     SmartDashboard.putNumber("HPAlign: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
        // }
    }

    @Override
    public void initialize() {
        int desiredTarget;
        if (DriverStation.isAutonomous()) {
            if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                desiredTarget = blueTargetTag;
            else
                desiredTarget = redTargetTag;
        }
        else
            desiredTarget = CalculateHPTarget.calculateTargetID();

        Pose2d tagPose = Limelight.getAprilTagPose(desiredTarget);
        double tagAngle = tagPose.getRotation().getRadians();
            
        SmartDashboard.putNumber("HPAlign: desired tag", desiredTarget);
        SmartDashboard.putNumber("HPAlign: tag angle", tagAngle);

        lateralOffset = SmartDashboard.getNumber("HPAlign: lateral offset", lateralOffset);
        backOffset = SmartDashboard.getNumber("HPAlign: back offset", backOffset);

        desiredPose = new Pose2d(
            tagPose.getX() + backOffset * Math.cos(tagAngle) + lateralOffset * Math.sin(tagAngle),
            tagPose.getY() + backOffset * Math.sin(tagAngle) - lateralOffset * Math.cos(tagAngle),
            new Rotation2d(0)
        );
        desiredAngle = tagPose.getRotation().getDegrees();

        xError = 10000;
        yError = 10000;
        rotationError = 10000;

        LimelightFrontMiddle.getInstance().setLED(Limelight.LightMode.ON);
        Logger.getInstance().logEvent("Align to HP, ID " + desiredTarget, true);

        Superstructure.getInstance().requestState(SuperstructureState.HP_INTAKE);
        
        {
            // TODO: use these values to determine what the path starting waypoint should be
            SmartDashboard.putNumber("HPAlign: " + "x", desiredPose.getX());
            SmartDashboard.putNumber("HPAlign: " + "y", desiredPose.getY());
        }
    }
    
    private Optional<Pose2d> getBestEstimatedPose() {
        for (Limelight camera : cameras) {
            Optional<Pose2d> measurement = camera.getEstimatedPoseMT2();
            if (measurement.isPresent() && camera.hasTarget())
                return Optional.of(measurement.get());
        }

        return Optional.empty();
        // return Optional.of(drivetrain.getPose());
    }

    private boolean translationDistanceGood() {
        // sqrt(xError^2 + yError^2) < 0.04
        return xError * xError + yError * yError < Math.pow(translateThreshold, 2);
    }

    @Override
    public void execute() {
        // {
        //     translateP = SmartDashboard.getNumber("HPAlign: translateP", translateP);
        //     translateI = SmartDashboard.getNumber("HPAlign: translateI", translateI);
        //     translateD = SmartDashboard.getNumber("HPAlign: translateD", translateD);
        //     translateFF = SmartDashboard.getNumber("HPAlign: translateFF", translateFF);
            translateThreshold = SmartDashboard.getNumber("HPAlign: translateThreshold", translateThreshold);

        //     rotationP = SmartDashboard.getNumber("HPAlign: rotationP", rotationP);
        //     rotationI = SmartDashboard.getNumber("HPAlign: rotationI", rotationI);
        //     rotationD = SmartDashboard.getNumber("HPAlign: rotationD", rotationD);
        //     rotationFF = SmartDashboard.getNumber("HPAlign: rotationFF", rotationFF);
        //     rotationThreshold = SmartDashboard.getNumber("HPAlign: rotationThreshold", rotationThreshold);
        //     rotationLowerP = SmartDashboard.getNumber("HPAlign: rotationLowerP", rotationLowerP);
        //     rotationUseLowerPThreshold = SmartDashboard.getNumber("HPAlign: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
        // }
        // maxSpeed = SmartDashboard.getNumber("HPAlign: max speed", maxSpeed);
        
        if (DriverStation.isAutonomous())
            rotationError = drivetrain.getHeadingBlueForceAdjust() - desiredAngle;
        else
            rotationError = drivetrain.getHeadingBlue() - desiredAngle;

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
        translation = MagnitudeCap.capMagnitude(translation, maxSpeed);

        if (DriverStation.isAutonomous())
            drivetrain.driveBlueForceAdjust(translation, rotation, true, null);
        else
            drivetrain.driveBlue(translation, rotation, true, null);

        Logger.getInstance().logAlignToHP(xError, yError, rotationError, xTranslate, yTranslate, rotation);

        {
            SmartDashboard.putNumber("HPAlign: estimatedX", estimatedPose.getX());
            SmartDashboard.putNumber("HPAlign: estimatedY", estimatedPose.getY());
            SmartDashboard.putNumber("HPAlign: xError", xError);
            SmartDashboard.putNumber("HPAlign: yError", yError);
            SmartDashboard.putNumber("HPAlign: rotationError", rotationError);

            SmartDashboard.putBoolean("HPAlign: translation good?", translationDistanceGood());
            SmartDashboard.putBoolean("HPAlign: rotation good?", Math.abs(rotationError) < rotationThreshold);
        }
    }

    @Override
    public void end(boolean interrupted) {
        LimelightFrontMiddle.getInstance().setLED(Limelight.LightMode.OFF);
        Logger.getInstance().logEvent(
            "Align to HP ended with errors: x " + xError + ", y " + yError + ", rotation " + rotationError,
            false
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rotationError) < rotationThreshold && translationDistanceGood();
    }
}
