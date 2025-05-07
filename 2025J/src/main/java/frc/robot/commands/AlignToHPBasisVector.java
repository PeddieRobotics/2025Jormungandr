package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFrontLeft;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.utils.CalculateHPTarget;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;
import frc.robot.utils.Logger;
import frc.robot.utils.MagnitudeCap;

public class AlignToHPBasisVector extends Command {
    private Drivetrain drivetrain;
    private Limelight[] cameras;

    private PIDController depthPIDController, lateralPIDController, rotationPIDController;

    private double depthP, depthI, depthD, depthFF;
    private double lateralP, lateralI, lateralD, lateralFF;

    private double rotationP, rotationI, rotationD, rotationFF;
    private double rotationLowerP, rotationUseLowerPThreshold;

    private double translateThreshold, autonomousTranslateThreshold, rotationThreshold;

    private double depthThreshold, lateralThreshold;

    private double maxSpeed;

    private Optional<Pose2d> desiredPose;
    private double desiredAngle;
    
    private double tagAngle;
    private Translation2d depthVector, lateralVector;

    private double xError, yError, rotationError, depthError, lateralError;

    private int blueTargetTag, redTargetTag;

    private double backOffset, lateralOffset;

    private double initialTime;

    public AlignToHPBasisVector(double maxSpeed, double lateralOffset, double backOffset, int blueTargetTag, int redTargetTag) {
        drivetrain = Drivetrain.getInstance();

        cameras = new Limelight[] {
            LimelightBack.getInstance(),
            LimelightFrontLeft.getInstance(),
            LimelightFrontRight.getInstance(),
        };

        depthP = HPAlign.kDepthP;
        depthI = HPAlign.kDepthI;
        depthD = HPAlign.kDepthD;
        depthFF = HPAlign.kDepthFF;

        lateralP = 3.0;
        lateralI = HPAlign.kLateralI;
        lateralD = HPAlign.kLateralD;
        lateralFF = HPAlign.kLateralFF;
        
        translateThreshold = HPAlign.kTranslateThreshold;
        autonomousTranslateThreshold = HPAlign.kAutoTranslateThreshold;

        rotationP = HPAlign.kRotationP;
        rotationI = HPAlign.kRotationI;
        rotationD = HPAlign.kRotationD;
        rotationFF = HPAlign.kRotationFF;
        rotationThreshold = HPAlign.kRotationThreshold;
        rotationLowerP = HPAlign.kRotationLowerP;
        rotationUseLowerPThreshold = HPAlign.kRotationUseLowerPThreshold;
        
        depthPIDController = new PIDController(depthP, depthI, depthD);
        lateralPIDController = new PIDController(lateralP, lateralI, lateralD);
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        rotationPIDController.enableContinuousInput(-180.0, 180.0);

        depthThreshold = HPAlign.kDepthThreshold;
        lateralThreshold = HPAlign.kLateralThreshold;
        
        this.maxSpeed = maxSpeed;
        this.lateralOffset = lateralOffset;
        this.backOffset = backOffset;
        this.blueTargetTag = blueTargetTag;
        this.redTargetTag = redTargetTag;
        
        tagAngle = 0;
        depthVector = new Translation2d();
        lateralVector = new Translation2d();

        addRequirements(drivetrain);

        SmartDashboard.putNumber("HPAlign: lateral offset", lateralOffset);
        SmartDashboard.putNumber("HPAlign: back offset", backOffset);

        SmartDashboard.putNumber("HPAlign: depthError", 0);
        SmartDashboard.putNumber("HPAlign: lateralError", 0);
        SmartDashboard.putNumber("HPAlign: depthMagnitude", 0);
        SmartDashboard.putNumber("HPAlign: lateralMagnitude", 0);


        SmartDashboard.putBoolean("HPAlign: rotation good", false);
        SmartDashboard.putBoolean("HPAlign: translation distance good", false);
        
        SmartDashboard.putNumber("HPAlign: depthP", depthP);
        SmartDashboard.putNumber("HPAlign: depthI", depthI);
        SmartDashboard.putNumber("HPAlign: depthD", depthD);
        SmartDashboard.putNumber("HPAlign: depthFF", depthFF);

        SmartDashboard.putNumber("HPAlign: lateralP", lateralP);
        SmartDashboard.putNumber("HPAlign: lateralI", lateralI);
        SmartDashboard.putNumber("HPAlign: lateralD", lateralD);
        SmartDashboard.putNumber("HPAlign: lateralFF", lateralFF);

        SmartDashboard.putNumber("HPAlign: translateThreshold", translateThreshold);
        SmartDashboard.putNumber("HPAlign: autonomousTranslateThreshold", autonomousTranslateThreshold);
        
        SmartDashboard.putNumber("HPAlign: rotationP", rotationP);
        SmartDashboard.putNumber("HPAlign: rotationI", rotationI);
        SmartDashboard.putNumber("HPAlign: rotationD", rotationD);
        SmartDashboard.putNumber("HPAlign: rotationFF", rotationFF);
        SmartDashboard.putNumber("HPAlign: rotationThreshold", rotationThreshold);
        SmartDashboard.putNumber("HPAlign: rotationLowerP", rotationLowerP);
        SmartDashboard.putNumber("HPAlign: rotationUseLowerPThreshold", rotationUseLowerPThreshold);

        SmartDashboard.putNumber("HPAlign: maxSpeed", maxSpeed);

        SmartDashboard.putNumber("HPAlign: Elapsed Time", 0.0);
        SmartDashboard.putNumber("HPAlign: depth auto threshold", HPAlign.kDepthThreshold);
        SmartDashboard.putNumber("HPAlign: lateral auto threshold", HPAlign.kLateralThreshold);
        SmartDashboard.putNumber("HPAlign: depth threshold", HPAlign.kDepthThreshold);
        SmartDashboard.putNumber("HPAlign: lateral threshold", HPAlign.kLateralThreshold);
    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        
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
        tagAngle = tagPose.getRotation().getRadians();

        SmartDashboard.putNumber("HPAlign: desired tag", desiredTarget);
        SmartDashboard.putNumber("HPAlign: tag angle", tagAngle);

        if (!DriverStation.isAutonomous()) {
            lateralOffset = SmartDashboard.getNumber("HPAlign: lateral offset", lateralOffset);
            backOffset = SmartDashboard.getNumber("HPAlign: back offset", backOffset);
        }

        desiredPose = Optional.of(new Pose2d(
            tagPose.getX() + backOffset * Math.cos(tagAngle) + lateralOffset * Math.sin(tagAngle),
            tagPose.getY() + backOffset * Math.sin(tagAngle) - lateralOffset * Math.cos(tagAngle),
            new Rotation2d(0)
        ));
        desiredAngle = tagPose.getRotation().getDegrees();

        SmartDashboard.putNumber("HPAlign: desired pose X", desiredPose.get().getX());
        SmartDashboard.putNumber("HPAlign: desired pose Y", desiredPose.get().getY());

        xError = 10000;
        yError = 10000;
        rotationError = 10000;
        depthError = 10000;
        lateralError = 10000;

        translateThreshold = HPAlign.kTranslateThreshold;

        depthVector = new Translation2d(Math.cos(tagAngle), Math.sin(tagAngle));
        lateralVector = new Translation2d(Math.sin(tagAngle), -Math.cos(tagAngle));
        
        Logger.getInstance().logEvent("Align to HP, ID " + desiredTarget, true);

        SmartDashboard.putNumber("HPAlign: target x", desiredPose.get().getX());
        SmartDashboard.putNumber("HPAlign: target y", desiredPose.get().getY());

        Superstructure.getInstance().requestState(SuperstructureState.HP_INTAKE);
    }
    
    private Optional<Pose2d> getBestEstimatedPose() {
        for (Limelight camera : cameras) {
            Optional<Pose2d> measurement = camera.getEstimatedPoseMT2();
            if (measurement.isPresent() && camera.hasTarget())
                return Optional.of(measurement.get());
        }

        // if (DriverStation.isAutonomous())
        //     return Optional.empty();
        return Optional.of(drivetrain.getPose());
    }
    
    private boolean translationDistanceGood() {
        // return xError * xError + yError * yError < Math.pow(translateThreshold, 2);
        // non-pythagoras based / reef align code:
        boolean depthErrorGood = Math.abs(depthError) < depthThreshold;
        boolean lateralErrorGood = Math.abs(lateralError) < lateralThreshold;
        return depthErrorGood && lateralErrorGood;
    }

    public boolean translationDistanceGoodInAuto(){
        // return xError * xError + yError * yError < Math.pow(autonomousTranslateThreshold, 2);
        boolean depthErrorGood = Math.abs(depthError) < depthThreshold;
        boolean lateralErrorGood = Math.abs(lateralError) < lateralThreshold;
        return depthErrorGood && lateralErrorGood;
    }

    public boolean depthGood(){
        return Math.abs(depthError) < SmartDashboard.getNumber("HPAlign: depth threshold", 0.0);
    }

    public boolean lateralGood(){
        return Math.abs(lateralError) < SmartDashboard.getNumber("HPAlign: lateral threshold", 0.0);
    }

    public boolean rotationGood(){
        return Math.abs(rotationError) < rotationThreshold;
    }

    @Override
    public void execute() {

        // depthThreshold = SmartDashboard.getNumber("HPAlign: depth threshold", 0.0);
        // lateralThreshold = SmartDashboard.getNumber("HPAlign: lateral threshold", 0.0);

        SmartDashboard.putBoolean("HPAlign: rotation good", rotationGood());
        SmartDashboard.putBoolean("HPAlign: translation distance good", translationDistanceGood());

        if(depthError != 10000 && lateralError != 10000){
            SmartDashboard.putNumber("HPAlign: depthError", depthError);
            SmartDashboard.putNumber("HPAlign: lateralError", lateralError);
        }

        SmartDashboard.putNumber("HPAlign: rotationError", rotationError);

        depthP = SmartDashboard.getNumber("HPAlign: depthP", depthP);
        depthI = SmartDashboard.getNumber("HPAlign: depthI", depthI);
        depthD = SmartDashboard.getNumber("HPAlign: depthD", depthD);
        depthFF = SmartDashboard.getNumber("HPAlign: depthFF", depthFF);

        lateralP = SmartDashboard.getNumber("HPAlign: lateralP", lateralP);
        lateralI = SmartDashboard.getNumber("HPAlign: lateralI", lateralI);
        lateralD = SmartDashboard.getNumber("HPAlign: lateralD", lateralD);
        lateralFF = SmartDashboard.getNumber("HPAlign: lateralFF", lateralFF);

        translateThreshold = SmartDashboard.getNumber("HPAlign: translateThreshold", translateThreshold);

        rotationP = SmartDashboard.getNumber("HPAlign: rotationP", rotationP);
        rotationI = SmartDashboard.getNumber("HPAlign: rotationI", rotationI);
        rotationD = SmartDashboard.getNumber("HPAlign: rotationD", rotationD);
        rotationFF = SmartDashboard.getNumber("HPAlign: rotationFF", rotationFF);
        rotationThreshold = SmartDashboard.getNumber("HPAlign: rotationThreshold", rotationThreshold);
        rotationLowerP = SmartDashboard.getNumber("HPAlign: rotationLowerP", rotationLowerP);
        rotationUseLowerPThreshold = SmartDashboard.getNumber("HPAlign: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
        
        maxSpeed = SmartDashboard.getNumber("HPAlign: maxSpeed", maxSpeed);
        
        rotationError = drivetrain.getHeadingBlue() - desiredAngle;

        depthPIDController.setPID(depthP, depthI, depthD);
        lateralPIDController.setPID(lateralP, lateralI, lateralD);

        if (Math.abs(rotationError) < rotationUseLowerPThreshold){
            rotationPIDController.setP(rotationLowerP);
        } else {
            rotationPIDController.setP(rotationP);
        }
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

        SmartDashboard.putNumber("HPAlign: xError", xError);
        SmartDashboard.putNumber("HPAlign: yError", yError);

        double depthMagnitude = 0, lateralMagnitude = 0;
        lateralError = xError * Math.sin(tagAngle) - yError * Math.cos(tagAngle);
        depthError = xError * Math.cos(tagAngle) + yError * Math.sin(tagAngle);

        if(!lateralGood()){
            lateralMagnitude = lateralPIDController.calculate(lateralError) + Math.signum(lateralError) * lateralFF;
        }

        if (!depthGood()){
            depthMagnitude = depthPIDController.calculate(depthError) + Math.signum(depthError) * depthFF;
        }

        // if (!translationDistanceGood()) {
        //     depthError = xError * Math.cos(tagAngle) + yError * Math.sin(tagAngle);
        //     lateralError = xError * Math.sin(tagAngle) - yError * Math.cos(tagAngle);

        //     depthMagnitude = depthPIDController.calculate(depthError) + Math.signum(depthError) * depthFF;
        //     lateralMagnitude = lateralPIDController.calculate(lateralError) + Math.signum(lateralError) * lateralFF;
        // }

        Translation2d translation = depthVector.times(depthMagnitude).plus(lateralVector.times(lateralMagnitude));
        translation = MagnitudeCap.capMagnitude(translation, maxSpeed);

        SmartDashboard.putNumber("HPAlign: translation output X", translation.getX());
        SmartDashboard.putNumber("HPAlign: translation output Y", translation.getY());

        drivetrain.driveBlue(translation, rotation, true, null);

        Logger.getInstance().logAlignToHP(lateralError, depthError, rotationError, lateralMagnitude, depthMagnitude, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        double elapsedTime = Timer.getFPGATimestamp()-initialTime;
        SmartDashboard.putNumber("HPAlign: Elapsed Time", elapsedTime);

        Logger.getInstance().logEvent(
            "Align to HP ended with errors: lateral " + lateralError + ", depth " + depthError + ", rotation " + rotationError,
            false
        );
    }

    @Override
    public boolean isFinished() {
        if (DriverStation.isAutonomousEnabled()) {
            if (Claw.getInstance().getTopSensor()) {
                return rotationGood() && Math.abs(depthError) < 0.12;
            }
            return rotationGood() && translationDistanceGoodInAuto();
        } else {
            return rotationGood() && translationDistanceGood();
        }
    }
}
