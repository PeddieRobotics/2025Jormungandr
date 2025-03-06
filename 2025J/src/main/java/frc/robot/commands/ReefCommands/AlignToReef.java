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

public class AlignToReef extends Command {
    private Drivetrain drivetrain;
    private Limelight[] cameras;

    private PIDController translatePIDController, rotationPIDController;

    private double translateP, translateI, translateD, translateFF, translateThreshold;
    private double rotationP, rotationI, rotationD, rotationFF, rotationThreshold;
    private double rotationLowerP, rotationUseLowerPThreshold;
    private double maxSpeed;

    private Optional<Pose2d> desiredPose;
    private double desiredAngle;

    private String commandName;
    private double tagBackMagnitude, tagLateralMagnitude;

    private double xError, yError, rotationError;

    private int blueTargetTag, redTargetTag;

    public AlignToReef(AlignmentDestination destination, double maxSpeed, int blueTargetTag, int redTargetTag) {
        drivetrain = Drivetrain.getInstance();
        
        // center
        switch (destination) {
            case LEFT -> {
                cameras = new Limelight[] {
                    LimelightFrontRight.getInstance(),
                    LimelightFrontMiddle.getInstance(),
                    LimelightFrontLeft.getInstance(),
                };
                commandName = "left align";
                tagLateralMagnitude = AlignmentConstants.ReefAlignEstimatedPose.kLeftOffset;
            }
            case MIDDLE -> {
                cameras = new Limelight[] {
                    LimelightFrontMiddle.getInstance(),
                    LimelightFrontLeft.getInstance(),
                    LimelightFrontRight.getInstance(),
                };
                commandName = "middle align";
                tagLateralMagnitude = AlignmentConstants.ReefAlignEstimatedPose.kMiddleOffset;
            }
            case RIGHT -> {
                cameras = new Limelight[] {
                    LimelightFrontLeft.getInstance(),
                    LimelightFrontMiddle.getInstance(),
                    LimelightFrontRight.getInstance(),
                };
                commandName = "right align";
                tagLateralMagnitude = AlignmentConstants.ReefAlignEstimatedPose.kRightOffset;
            }
        }

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
        
        tagBackMagnitude = ReefAlignEstimatedPose.kTagBackMagnitude;
        
        this.maxSpeed = maxSpeed;
        this.blueTargetTag = blueTargetTag;
        this.redTargetTag = redTargetTag;

        SmartDashboard.putNumber(commandName + " lateral offset", tagLateralMagnitude);
        SmartDashboard.putNumber(commandName + " back offset", tagBackMagnitude);

        addRequirements(drivetrain);
        
        // {
        //     SmartDashboard.putNumber("Align: translateP", translateP);
        //     SmartDashboard.putNumber("Align: translateI", translateI);
        //     SmartDashboard.putNumber("Align: translateD", translateD);
        //     SmartDashboard.putNumber("Align: translateFF", translateFF);
        //     SmartDashboard.putNumber("Align: translateThreshold", translateThreshold);
            
        //     SmartDashboard.putNumber("Align: rotationP", rotationP);
        //     SmartDashboard.putNumber("Align: rotationI", rotationI);
        //     SmartDashboard.putNumber("Align: rotationD", rotationD);
        //     SmartDashboard.putNumber("Align: rotationFF", rotationFF);
        //     SmartDashboard.putNumber("Align: rotationThreshold", rotationThreshold);
        //     SmartDashboard.putNumber("Align: rotationLowerP", rotationLowerP);
        //     SmartDashboard.putNumber("Align: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
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
        else if (SmartDashboard.getBoolean("Align: Smart Target Finding", true)) {
            // if aligning from unacceptable position ("sad face case"): returns 0
            desiredTarget = CalculateReefTarget.calculateTargetID();
        }
        else
            desiredTarget = LimelightFrontMiddle.getInstance().getTargetID();

        // SmartDashboard.putNumber("Align: Desired Target", desiredTarget);

        if (!AlignmentConstants.kReefDesiredAngle.containsKey(desiredTarget)) {
            desiredPose = Optional.empty();
            return;
        }
        desiredAngle = AlignmentConstants.kReefDesiredAngle.get(desiredTarget);
        
        Pose2d tagPose = Limelight.getAprilTagPose(desiredTarget);
        double tagAngle = tagPose.getRotation().getRadians();

        tagLateralMagnitude = SmartDashboard.getNumber(commandName + " lateral offset", tagLateralMagnitude);
        tagBackMagnitude = SmartDashboard.getNumber(commandName + " back offset", tagBackMagnitude);

        desiredPose = Optional.of(new Pose2d(
            tagPose.getX() + tagBackMagnitude * Math.cos(tagAngle) + tagLateralMagnitude * Math.sin(tagAngle),
            tagPose.getY() + tagBackMagnitude * Math.sin(tagAngle) - tagLateralMagnitude * Math.cos(tagAngle),
            new Rotation2d(0)
        ));

        xError = 10000;
        yError = 10000;
        rotationError = 10000;

        LimelightFrontMiddle.getInstance().setLED(Limelight.LightMode.ON);
        // SmartDashboard.putBoolean("Align: Finished?", false);
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
        // {
        //     translateP = SmartDashboard.getNumber("Align: translateP", translateP);
        //     translateI = SmartDashboard.getNumber("Align: translateI", translateI);
        //     translateD = SmartDashboard.getNumber("Align: translateD", translateD);
        //     translateFF = SmartDashboard.getNumber("Align: translateFF", translateFF);
        //     translateThreshold = SmartDashboard.getNumber("Align: translateThreshold", translateThreshold);

        //     rotationP = SmartDashboard.getNumber("Align: rotationP", rotationP);
        //     rotationI = SmartDashboard.getNumber("Align: rotationI", rotationI);
        //     rotationD = SmartDashboard.getNumber("Align: rotationD", rotationD);
        //     rotationFF = SmartDashboard.getNumber("Align: rotationFF", rotationFF);
        //     rotationThreshold = SmartDashboard.getNumber("Align: rotationThreshold", rotationThreshold);
        //     rotationLowerP = SmartDashboard.getNumber("Align: rotationLowerP", rotationLowerP);
        //     rotationUseLowerPThreshold = SmartDashboard.getNumber("Align: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
        // }
        
        if (desiredPose.isEmpty())
            return;

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

        xError = estimatedPose.getX() - desiredPose.get().getX();
        yError = estimatedPose.getY() - desiredPose.get().getY();
        
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

        boolean autoScore = SmartDashboard.getBoolean("Align: Auto Score", true);
        if (Math.abs(rotationError) < rotationThreshold && translationDistanceGood() && autoScore)
            Superstructure.getInstance().sendToScore();

        // {
        //     SmartDashboard.putNumber("Align: xError", xError);
        //     SmartDashboard.putNumber("Align: yError", yError);
        //     SmartDashboard.putNumber("Align: rotationError", rotationError);

        //     SmartDashboard.putBoolean("Align: translation good?", translationDistanceGood());
        //     SmartDashboard.putBoolean("Align: rotation good?", Math.abs(rotationError) < rotationThreshold);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        if (desiredPose.isPresent())
            drivetrain.drive(new Translation2d(0,0), 0, false, null);
        
        LimelightFrontMiddle.getInstance().setLED(Limelight.LightMode.OFF);
        // SmartDashboard.putBoolean("Align: Finished?", true);
    }

    @Override
    public boolean isFinished() {
        return desiredPose.isEmpty() || Superstructure.getInstance().isReefScoringState();
    }
}
