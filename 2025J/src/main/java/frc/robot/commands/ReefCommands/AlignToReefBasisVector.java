package frc.robot.commands.ReefCommands;

import static frc.robot.subsystems.Superstructure.SuperstructureState.HP_INTAKE;
import static frc.robot.subsystems.Superstructure.SuperstructureState.L4_PREP;
import static frc.robot.subsystems.Superstructure.SuperstructureState.STOW;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFrontLeft;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.utils.CalculateReefTarget;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlign;
import frc.robot.utils.Logger;
import frc.robot.utils.MagnitudeCap;

public class AlignToReefBasisVector extends Command {
    private Drivetrain drivetrain;
    private Limelight[] cameras;

    private PIDController depthPIDController, lateralPIDController, rotationPIDController;

    private double depthP, depthI, depthD, depthFF;

    private double depthThreshold, depthCloseThreshold, depthCloseAtL4Threshold;

    private double lateralP, lateralI, lateralD, lateralFF;

    private double lateralThreshold, lateralCloseThreshold, lateralCloseAtL4Threshold;

    private double rotationP, rotationI, rotationD, rotationFF;
    private double rotationLowerP, rotationUseLowerPThreshold;

    private double rotationThreshold;

    private double maxSpeed;

    private Optional<Pose2d> desiredPose;
    private double desiredAngle;
    
    private double tagAngle;
    private Translation2d depthVector, lateralVector;

    private String commandName;
    private double tagBackMagnitude, tagLateralMagnitude;

    private double xError, yError, rotationError, depthError, lateralError;

    private int blueTargetTag, redTargetTag;

    private double initialTime;

    public AlignToReefBasisVector(AlignmentDestination destination, double maxSpeed, int blueTargetTag, int redTargetTag) {
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
                tagLateralMagnitude = AlignmentConstants.ReefAlign.kLeftOffset;
            }
            case MIDDLE -> {
                cameras = new Limelight[] {
                    LimelightFrontMiddle.getInstance(),
                    LimelightFrontLeft.getInstance(),
                    LimelightFrontRight.getInstance(),
                };
                commandName = "middle align";
                tagLateralMagnitude = AlignmentConstants.ReefAlign.kMiddleOffset;
            }
            case RIGHT -> {
                cameras = new Limelight[] {
                    LimelightFrontLeft.getInstance(),
                    LimelightFrontMiddle.getInstance(),
                    LimelightFrontRight.getInstance(),
                };
                commandName = "right align";
                tagLateralMagnitude = AlignmentConstants.ReefAlign.kRightOffset;
            }
        }

        depthP = ReefAlign.kDepthP;
        depthI = ReefAlign.kDepthI;
        depthD = ReefAlign.kDepthD;
        depthFF = ReefAlign.kDepthFF;

        depthThreshold = ReefAlign.kDepthThreshold;
        depthCloseThreshold = ReefAlign.kDepthCloseThreshold;
        depthCloseAtL4Threshold = ReefAlign.kDepthCloseAtL4Threshold;

        lateralP = ReefAlign.kLateralP;
        lateralI = ReefAlign.kLateralI;
        lateralD = ReefAlign.kLateralD;
        lateralFF = ReefAlign.kLateralFF;
        
        lateralThreshold = ReefAlign.kLateralThreshold;
        lateralCloseThreshold = ReefAlign.kLateralCloseThreshold;
        lateralCloseAtL4Threshold = ReefAlign.kLateralCloseAtL4Threshold;

        rotationP = ReefAlign.kRotationP;
        rotationI = ReefAlign.kRotationI;
        rotationD = ReefAlign.kRotationD;
        rotationFF = ReefAlign.kRotationFF;
        rotationThreshold = ReefAlign.kRotationThreshold;
        rotationLowerP = ReefAlign.kRotationLowerP;
        rotationUseLowerPThreshold = ReefAlign.kRotationUseLowerPThreshold;
        
        depthPIDController = new PIDController(depthP, depthI, depthD);
        lateralPIDController = new PIDController(lateralP, lateralI, lateralD);
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        rotationPIDController.enableContinuousInput(-180.0, 180.0);
        
        tagBackMagnitude = ReefAlign.kTagBackMagnitude;
        
        tagAngle = 0;
        depthVector = new Translation2d();
        lateralVector = new Translation2d();
        
        this.maxSpeed = maxSpeed;
        this.blueTargetTag = blueTargetTag;
        this.redTargetTag = redTargetTag;

        SmartDashboard.putNumber(commandName + " lateral offset", tagLateralMagnitude);
        SmartDashboard.putNumber(commandName + " back offset", tagBackMagnitude);

        addRequirements(drivetrain);
        
        {
            SmartDashboard.putNumber("Align: depthP", depthP);
            SmartDashboard.putNumber("Align: depthI", depthI);
            SmartDashboard.putNumber("Align: depthD", depthD);
            SmartDashboard.putNumber("Align: depthFF", depthFF);

            SmartDashboard.putNumber("Align: lateralP", lateralP);
            SmartDashboard.putNumber("Align: lateralI", lateralI);
            SmartDashboard.putNumber("Align: lateralD", lateralD);
            SmartDashboard.putNumber("Align: lateralFF", lateralFF);
            
            SmartDashboard.putNumber("Align: rotationP", rotationP);
            SmartDashboard.putNumber("Align: rotationI", rotationI);
            SmartDashboard.putNumber("Align: rotationD", rotationD);
            SmartDashboard.putNumber("Align: rotationFF", rotationFF);
            SmartDashboard.putNumber("Align: rotationThreshold", rotationThreshold);
            SmartDashboard.putNumber("Align: rotationLowerP", rotationLowerP);
            SmartDashboard.putNumber("Align: rotationUseLowerPThreshold", rotationUseLowerPThreshold);

            SmartDashboard.putNumber("Align: maxSpeed", maxSpeed);
        }

        SmartDashboard.putNumber("Align: Elapsed Time", 0.0);

        SmartDashboard.putNumber("Align: depth close threshold", ReefAlign.kDepthCloseThreshold);
        SmartDashboard.putNumber("Align: depth close at L4 threshold", ReefAlign.kDepthCloseAtL4Threshold);
        SmartDashboard.putNumber("Align: lateral close threshold", ReefAlign.kLateralCloseThreshold);
        SmartDashboard.putNumber("Align: lateral close at L4 threshold", ReefAlign.kLateralCloseAtL4Threshold);

        SmartDashboard.putNumber("Align: depth threshold", ReefAlign.kDepthThreshold);
        SmartDashboard.putNumber("Align: lateral threshold", ReefAlign.kLateralThreshold);

        SmartDashboard.putBoolean("Align: fire gamepiece", false);

        SmartDashboard.putNumber("Align: depthError", depthError);
        SmartDashboard.putNumber("Align: lateralError", lateralError);

        SmartDashboard.putBoolean("HPAlign: rotation good", false);
        SmartDashboard.putBoolean("HPAlign: depth good", false);
        SmartDashboard.putBoolean("HPAlign: lateral good", false);
        SmartDashboard.putBoolean("HPAlign: depth close", false);
        SmartDashboard.putBoolean("HPAlign: lateral close", false);

    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        SmartDashboard.putBoolean("Align: fire gamepiece", false);

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

        SmartDashboard.putNumber("Align: Desired Target", desiredTarget);

        if (!AlignmentConstants.kReefDesiredAngle.containsKey(desiredTarget)) {
            desiredPose = Optional.empty();
            return;
        }
        desiredAngle = AlignmentConstants.kReefDesiredAngle.get(desiredTarget);
        
        Pose2d tagPose = Limelight.getAprilTagPose(desiredTarget);
        tagAngle = tagPose.getRotation().getRadians();

        tagLateralMagnitude = SmartDashboard.getNumber(commandName + " lateral offset", tagLateralMagnitude);
        tagBackMagnitude = SmartDashboard.getNumber(commandName + " back offset", tagBackMagnitude);

        if (Superstructure.getInstance().getCurrentState() == SuperstructureState.L1_PREP){
            // if (commandName.equals("left align")) {
            //     tagLateralMagnitude = SmartDashboard.getNumber("L1AlignLeft: lateral offset", 0.1);
            //     tagBackMagnitude = SmartDashboard.getNumber("L1AlignLeft: back offset", 0.55);
            //     desiredAngle += SmartDashboard.getNumber("L1AlignLeft: angle offset", 20.0);
            // }
            // else if (commandName.equals("right align")) {
            //     tagLateralMagnitude = SmartDashboard.getNumber("L1AlignRight: lateral offset", -0.1);
            //     tagBackMagnitude = SmartDashboard.getNumber("L1AlignRight: back offset", 0.55);
            //     desiredAngle += SmartDashboard.getNumber("L1AlignRight: angle offset", -20.0);
            // }

            tagLateralMagnitude = 0.0;
        }

        desiredPose = Optional.of(new Pose2d(
            tagPose.getX() + tagBackMagnitude * Math.cos(tagAngle) + tagLateralMagnitude * Math.sin(tagAngle),
            tagPose.getY() + tagBackMagnitude * Math.sin(tagAngle) - tagLateralMagnitude * Math.cos(tagAngle),
            new Rotation2d(0)
        ));
        
        depthVector = new Translation2d(Math.cos(tagAngle), Math.sin(tagAngle));
        lateralVector = new Translation2d(Math.sin(tagAngle), -Math.cos(tagAngle));

        xError = 10000;
        yError = 10000;
        rotationError = 10000;
        depthError = 10000;
        lateralError = 10000;

        Logger.getInstance().logEvent("Align to Reef, ID " + desiredTarget, true);
    }
    
    private Optional<Pose2d> getBestEstimatedPose() {
        for (Limelight camera : cameras) {
            Optional<Pose2d> measurement = camera.getEstimatedPoseMT2();
            if (measurement.isPresent() && camera.hasTarget())
                return Optional.of(measurement.get());
        }

        if (DriverStation.isAutonomous())
            return Optional.empty();
        return Optional.of(drivetrain.getPose());

        // return Optional.empty();
    }

    private boolean depthClose(){
        if(Superstructure.getInstance().getCurrentState() == L4_PREP){
            return Math.abs(depthError) < depthCloseAtL4Threshold;
        }
        else{
            return Math.abs(depthError) < depthCloseThreshold;
        }

    }

    private boolean lateralClose(){
        if(Superstructure.getInstance().getCurrentState() == L4_PREP){
            return Math.abs(lateralError) < lateralCloseAtL4Threshold;
        }
        else{
            return Math.abs(depthError) < lateralCloseThreshold;
        }
    }

    private boolean depthAndLateralClose() {
        return depthClose() && lateralClose();
    }

    private boolean depthGood(){
        return Math.abs(depthError) < depthThreshold;
    }

    private boolean lateralGood(){
        return Math.abs(lateralError) < lateralThreshold;
    }

    private boolean depthAndLateralGood() {
        return depthGood() && lateralGood();
    }

    private boolean rotationGood(){
        return Math.abs(rotationError) < rotationThreshold;
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("HPAlign: rotation good", rotationGood());
        SmartDashboard.putBoolean("HPAlign: depth good", depthGood());
        SmartDashboard.putBoolean("HPAlign: lateral good", lateralGood());
        SmartDashboard.putBoolean("HPAlign: depth close", depthClose());
        SmartDashboard.putBoolean("HPAlign: lateral close", lateralClose());

        {
            depthP = SmartDashboard.getNumber("Align: depthP", depthP);
            depthI = SmartDashboard.getNumber("Align: depthI", depthI);
            depthD = SmartDashboard.getNumber("Align: depthD", depthD);
            depthFF = SmartDashboard.getNumber("Align: depthFF", depthFF);

            lateralP = SmartDashboard.getNumber("Align: lateralP", lateralP);
            lateralI = SmartDashboard.getNumber("Align: lateralI", lateralI);
            lateralD = SmartDashboard.getNumber("Align: lateralD", lateralD);
            lateralFF = SmartDashboard.getNumber("Align: lateralFF", lateralFF);

            rotationP = SmartDashboard.getNumber("Align: rotationP", rotationP);
            rotationI = SmartDashboard.getNumber("Align: rotationI", rotationI);
            rotationD = SmartDashboard.getNumber("Align: rotationD", rotationD);
            rotationFF = SmartDashboard.getNumber("Align: rotationFF", rotationFF);
            rotationThreshold = SmartDashboard.getNumber("Align: rotationThreshold", rotationThreshold);
            rotationLowerP = SmartDashboard.getNumber("Align: rotationLowerP", rotationLowerP);
            rotationUseLowerPThreshold = SmartDashboard.getNumber("Align: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
            
            maxSpeed = SmartDashboard.getNumber("Align: maxSpeed", maxSpeed);

            depthCloseThreshold = SmartDashboard.getNumber("Align: depth close threshold", 0.0);
            depthCloseAtL4Threshold = SmartDashboard.getNumber("Align: depth close at L4 threshold", 0.0);
            lateralCloseThreshold = SmartDashboard.getNumber("Align: lateral close threshold", 0.0);
            lateralCloseAtL4Threshold = SmartDashboard.getNumber("Align: lateral close at L4 threshold", 0.0);
            depthThreshold = SmartDashboard.getNumber("Align: depth threshold", 0.0);
            lateralThreshold = SmartDashboard.getNumber("Align: lateral threshold", 0.0);
        }
        
        if (desiredPose.isEmpty())
            return;

        if (DriverStation.isAutonomous())
            rotationError = drivetrain.getHeadingBlueForceAdjust() - desiredAngle;
        else
            rotationError = drivetrain.getHeadingBlue() - desiredAngle;

        depthPIDController.setPID(depthP, depthI, depthD);
        lateralPIDController.setPID(lateralP, lateralI, lateralD);

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
        double depthMagnitude = 0, lateralMagnitude = 0;
        if (!depthAndLateralGood()) {
            depthError = xError * Math.cos(tagAngle) + yError * Math.sin(tagAngle);
            lateralError = xError * Math.sin(tagAngle) - yError * Math.cos(tagAngle);

            depthMagnitude = depthPIDController.calculate(depthError) + Math.signum(depthError) * depthFF;
            lateralMagnitude = lateralPIDController.calculate(lateralError) + Math.signum(lateralError) * lateralFF;
        }
        SmartDashboard.putNumber("Align: depthError", depthError);
        SmartDashboard.putNumber("Align: lateralError", lateralError);

        Translation2d translation = depthVector.times(depthMagnitude).plus(lateralVector.times(lateralMagnitude));
        translation = MagnitudeCap.capMagnitude(translation, maxSpeed);

        if (DriverStation.isAutonomous())
            drivetrain.driveBlueForceAdjust(translation, rotation, true, null);
        else
            drivetrain.driveBlue(translation, rotation, true, null);

        boolean autoScore = SmartDashboard.getBoolean("Align: Auto Score", true);

        double elapsedTime = Timer.getFPGATimestamp() - initialTime;
        if (Math.abs(rotationError) < rotationThreshold && depthAndLateralClose() && autoScore && (elapsedTime > 0.3 || depthAndLateralGood())) {
            Superstructure.getInstance().sendToScore();
            SmartDashboard.putBoolean("Align: fire gamepiece", true);
            if(Superstructure.getInstance().isReefScoringState()){
                LimelightFrontMiddle.getInstance().flashLED();
                LimelightBack.getInstance().flashLED();
                SmartDashboard.putNumber("Align: Elapsed Time", elapsedTime);
            }
        }

        Logger.getInstance().logAlignToReef(xError, yError, rotationError, depthMagnitude, lateralMagnitude, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        if (desiredPose.isPresent())
            drivetrain.drive(new Translation2d(0,0), 0, false, null);
  
        Logger.getInstance().logEvent(
            "Align to Reef ended with errors: x " + xError + ", y " + yError + ", rotation " + rotationError,
            false
        );
    }

    @Override
    public boolean isFinished() {
        return desiredPose.isEmpty() || ((Superstructure.getInstance().isReefScoringState() || Superstructure.getInstance().getCurrentState() == HP_INTAKE || Superstructure.getInstance().getCurrentState() == STOW) 
        && rotationGood() && depthAndLateralGood());
    }
}
