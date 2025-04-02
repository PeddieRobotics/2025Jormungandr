package frc.robot.commands;

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
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ScoringFlag;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.utils.CalculateReefTarget;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlign;
import frc.robot.utils.Logger;
import frc.robot.utils.MagnitudeCap;
import frc.robot.utils.PoleLookup;

public class AlignToReefBasisVector extends Command {
    private Drivetrain drivetrain;
    private Limelight[] cameras;

    private PIDController depthPIDController, lateralPIDController, rotationPIDController;

    private double depthP, depthI, depthD, depthFF;

    private double depthThreshold, depthCloseThreshold, depthCloseAtL4Threshold, depthL3PrestageThreshold, depthL4PrestageThreshold;

    private double lateralP, lateralI, lateralD, lateralFF;

    private double lateralThreshold, lateralCloseThreshold, lateralCloseAtL4Threshold, lateralL3PrestageThreshold, lateralL4PrestageThreshold;

    private double rotationP, rotationI, rotationD, rotationFF;
    private double rotationLowerP, rotationUseLowerPThreshold;

    private double rotationThreshold;

    private double maxSpeed;

    private Optional<Pose2d> desiredPose;
    private double desiredAngle;
    
    private double tagAngle;
    private Translation2d depthVector, lateralVector;

    private String commandName;
    
    private double teleopBackOffset, autoBackOffset;
    private double tagBackMagnitude, tagLateralMagnitude;

    private double xError, yError, rotationError, depthError, lateralError;

    private int blueTargetTag, redTargetTag;

    private double initialTime;

    private double L4offset;

    private AlignmentDestination destination;
    
    public AlignToReefBasisVector(AlignmentDestination destination, double maxSpeed, double autoBackOffset, double teleopBackOffset, int blueTargetTag, int redTargetTag, double L4offset) {
        drivetrain = Drivetrain.getInstance();
        
        // center
        switch (destination) {
            case LEFT -> {
                cameras = new Limelight[] {
                    LimelightFrontRight.getInstance(),
                    LimelightFrontLeft.getInstance(),
                };
                commandName = "left align";
                tagLateralMagnitude = AlignmentConstants.ReefAlign.kLeftOffset;
            }
            case MIDDLE -> {
                cameras = new Limelight[] {
                    LimelightFrontLeft.getInstance(),
                    LimelightFrontRight.getInstance(),
                };
                commandName = "middle align";
                tagLateralMagnitude = AlignmentConstants.ReefAlign.kMiddleOffset;
            }
            case RIGHT -> {
                cameras = new Limelight[] {
                    LimelightFrontLeft.getInstance(),
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

        depthL3PrestageThreshold = ReefAlign.kDepthL3PrestageThreshold;
        depthL4PrestageThreshold = ReefAlign.kDepthL4PrestageThreshold;

        lateralP = ReefAlign.kLateralP;
        lateralI = ReefAlign.kLateralI;
        lateralD = ReefAlign.kLateralD;
        lateralFF = ReefAlign.kLateralFF;
        
        lateralThreshold = ReefAlign.kLateralThreshold;
        lateralCloseThreshold = ReefAlign.kLateralCloseThreshold;
        lateralCloseAtL4Threshold = ReefAlign.kLateralCloseAtL4Threshold;

        lateralL3PrestageThreshold = ReefAlign.kLateralL3PrestageThreshold;
        lateralL4PrestageThreshold = ReefAlign.kLateralL4PrestageThreshold;

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
        
        tagAngle = 0;
        depthVector = new Translation2d();
        lateralVector = new Translation2d();
        
        this.maxSpeed = maxSpeed;
        this.blueTargetTag = blueTargetTag;
        this.redTargetTag = redTargetTag;
        this.autoBackOffset = autoBackOffset;
        this.teleopBackOffset = teleopBackOffset;
        this.L4offset = L4offset;
        this.destination = destination;

        tagBackMagnitude = teleopBackOffset; // ReefAlign.kTagBackMagnitude;
        
        SmartDashboard.putNumber(commandName + " lateral offset", tagLateralMagnitude);
        SmartDashboard.putNumber(commandName + " back offset", tagBackMagnitude);

        addRequirements(drivetrain);
        
        // {
        //     SmartDashboard.putNumber("Align: depthP", depthP);
        //     SmartDashboard.putNumber("Align: depthI", depthI);
        //     SmartDashboard.putNumber("Align: depthD", depthD);
        //     SmartDashboard.putNumber("Align: depthFF", depthFF);

        //     SmartDashboard.putNumber("Align: lateralP", lateralP);
        //     SmartDashboard.putNumber("Align: lateralI", lateralI);
        //     SmartDashboard.putNumber("Align: lateralD", lateralD);
        //     SmartDashboard.putNumber("Align: lateralFF", lateralFF);
            
        //     SmartDashboard.putNumber("Align: rotationP", rotationP);
        //     SmartDashboard.putNumber("Align: rotationI", rotationI);
        //     SmartDashboard.putNumber("Align: rotationD", rotationD);
        //     SmartDashboard.putNumber("Align: rotationFF", rotationFF);
        //     SmartDashboard.putNumber("Align: rotationThreshold", rotationThreshold);
        //     SmartDashboard.putNumber("Align: rotationLowerP", rotationLowerP);
        //     SmartDashboard.putNumber("Align: rotationUseLowerPThreshold", rotationUseLowerPThreshold);

        //     SmartDashboard.putNumber("Align: maxSpeed", maxSpeed);
        // }

        // SmartDashboard.putNumber("Align: Elapsed Time", 0.0);

        // SmartDashboard.putNumber("Align: depth close threshold", ReefAlign.kDepthCloseThreshold);
        // SmartDashboard.putNumber("Align: depth close at L4 threshold", ReefAlign.kDepthCloseAtL4Threshold);
        // SmartDashboard.putNumber("Align: depth L3 prestage threshold", ReefAlign.kDepthL3PrestageThreshold);
        // SmartDashboard.putNumber("Align: depth L4 prestage threshold", ReefAlign.kDepthL4PrestageThreshold);

        // SmartDashboard.putNumber("Align: lateral close threshold", ReefAlign.kLateralCloseThreshold);
        // SmartDashboard.putNumber("Align: lateral close at L4 threshold", ReefAlign.kLateralCloseAtL4Threshold);
        // SmartDashboard.putNumber("Align: lateral L3 prestage threshold", ReefAlign.kLateralL3PrestageThreshold);
        // SmartDashboard.putNumber("Align: lateral L4 prestage threshold", ReefAlign.kLateralL4PrestageThreshold);

        // SmartDashboard.putNumber("Align: depth threshold", ReefAlign.kDepthThreshold);
        // SmartDashboard.putNumber("Align: lateral threshold", ReefAlign.kLateralThreshold);

        // SmartDashboard.putBoolean("Align: fire gamepiece", false);

        // SmartDashboard.putNumber("Align: depthError", depthError);
        // SmartDashboard.putNumber("Align: lateralError", lateralError);

        // SmartDashboard.putBoolean("Align: rotation good", false);
        // SmartDashboard.putBoolean("Align: depth good", false);
        // SmartDashboard.putBoolean("Align: lateral good", false);
        // SmartDashboard.putBoolean("Align: depth close", false);
        // SmartDashboard.putBoolean("Align: lateral close", false);
    }

    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        this.L4offset = 0;
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
            desiredTarget = LimelightFrontLeft.getInstance().getTargetID();

        SmartDashboard.putNumber("Align: Desired Target", desiredTarget);

        if (!AlignmentConstants.kReefDesiredAngle.containsKey(desiredTarget)) {
            desiredPose = Optional.empty();
            return;
        }
        desiredAngle = AlignmentConstants.kReefDesiredAngle.get(desiredTarget);
        
        Pose2d tagPose = Limelight.getAprilTagPose(desiredTarget);
        tagAngle = tagPose.getRotation().getRadians();

        tagLateralMagnitude = SmartDashboard.getNumber(commandName + " lateral offset", tagLateralMagnitude);

        if (DriverStation.isAutonomous()) {
            tagBackMagnitude = autoBackOffset; // ReefAlign.kAutoTagBackMagnitude;
            depthP = ReefAlign.kAutoDepthP;
            lateralP = ReefAlign.kAutoLateralP;
        }
        else {
            tagBackMagnitude = teleopBackOffset; // SmartDashboard.getNumber(commandName + " back offset", tagBackMagnitude);
            depthP = ReefAlign.kDepthP;
            lateralP = ReefAlign.kLateralP;
        }

        depthPIDController.setP(depthP);
        lateralPIDController.setP(lateralP);

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

        Logger.getInstance().logEvent("Reef " + commandName + ", ID " + desiredTarget + ", L4 Offset " + L4offset, true);

        double offset = PoleLookup.lookupPole(desiredTarget, destination);
        Superstructure.getInstance().setL4offset(offset);
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

    private boolean l3PrepSafe(){
        return Math.abs(depthError) < depthL3PrestageThreshold && Math.abs(lateralError) < lateralL3PrestageThreshold;
    }

    private boolean l4PrepSafe(){
        return Math.abs(depthError) < depthL4PrestageThreshold && Math.abs(lateralError) < lateralL4PrestageThreshold;
    }

    @Override
    public void execute() {
        // SmartDashboard.putBoolean("Align: rotation good", rotationGood());
        // SmartDashboard.putBoolean("Align: depth good", depthGood());
        // SmartDashboard.putBoolean("Align: lateral good", lateralGood());
        // SmartDashboard.putBoolean("Align: depth close", depthClose());
        // SmartDashboard.putBoolean("Align: lateral close", lateralClose());

        // SmartDashboard.putBoolean("Align: l3PrepSafe", l3PrepSafe());
        // SmartDashboard.putBoolean("Align: l4PrepSafe", l4PrepSafe());

        // {
        //     depthP = SmartDashboard.getNumber("Align: depthP", depthP);
        //     depthI = SmartDashboard.getNumber("Align: depthI", depthI);
        //     depthD = SmartDashboard.getNumber("Align: depthD", depthD);
        //     depthFF = SmartDashboard.getNumber("Align: depthFF", depthFF);

        //     lateralP = SmartDashboard.getNumber("Align: lateralP", lateralP);
        //     lateralI = SmartDashboard.getNumber("Align: lateralI", lateralI);
        //     lateralD = SmartDashboard.getNumber("Align: lateralD", lateralD);
        //     lateralFF = SmartDashboard.getNumber("Align: lateralFF", lateralFF);

        //     rotationP = SmartDashboard.getNumber("Align: rotationP", rotationP);
        //     rotationI = SmartDashboard.getNumber("Align: rotationI", rotationI);
        //     rotationD = SmartDashboard.getNumber("Align: rotationD", rotationD);
        //     rotationFF = SmartDashboard.getNumber("Align: rotationFF", rotationFF);
        //     rotationThreshold = SmartDashboard.getNumber("Align: rotationThreshold", rotationThreshold);
        //     rotationLowerP = SmartDashboard.getNumber("Align: rotationLowerP", rotationLowerP);
        //     rotationUseLowerPThreshold = SmartDashboard.getNumber("Align: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
            
        //     maxSpeed = SmartDashboard.getNumber("Align: maxSpeed", maxSpeed);

        //     depthCloseThreshold = SmartDashboard.getNumber("Align: depth close threshold", 0.0);
        //     depthCloseAtL4Threshold = SmartDashboard.getNumber("Align: depth close at L4 threshold", 0.0);
        //     lateralCloseThreshold = SmartDashboard.getNumber("Align: lateral close threshold", 0.0);
        //     lateralCloseAtL4Threshold = SmartDashboard.getNumber("Align: lateral close at L4 threshold", 0.0);
        //     depthThreshold = SmartDashboard.getNumber("Align: depth threshold", 0.0);
        //     lateralThreshold = SmartDashboard.getNumber("Align: lateral threshold", 0.0);

        //     depthL3PrestageThreshold = SmartDashboard.getNumber("Align: depth L3 prestage threshold", 0.0);
        //     depthL4PrestageThreshold = SmartDashboard.getNumber("Align: depth L4 prestage threshold", 0.0);

        //     lateralL3PrestageThreshold = SmartDashboard.getNumber("Align: lateral L3 prestage threshold", 0.0);
        //     lateralL4PrestageThreshold = SmartDashboard.getNumber("Align: lateral L4 prestage threshold", 0.0);
        // }
        
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

        double elapsedTime = Timer.getFPGATimestamp() - initialTime;

        // Auto Prep State Logic
        boolean autoPrep = SmartDashboard.getBoolean("Align: Auto Prep", true);

        if(!DriverStation.isAutonomousEnabled() && autoPrep){
            if(elapsedTime > 0.05 && (Superstructure.getInstance().getCurrentState() == SuperstructureState.PRESTAGE || Superstructure.getInstance().isReefPrepState()) && Superstructure.getInstance().getScoringFlag() == ScoringFlag.L2FLAG){
                Logger.getInstance().logEvent("Align to reef to L2 Prep", true);
                Superstructure.getInstance().requestState(SuperstructureState.L2_PREP);
            }

            if(elapsedTime > 0.05 && l3PrepSafe() && (Superstructure.getInstance().getCurrentState() == SuperstructureState.PRESTAGE || Superstructure.getInstance().isReefPrepState()) && Superstructure.getInstance().getScoringFlag() == ScoringFlag.L3FLAG){
                Logger.getInstance().logEvent("Align to reef to L3 Prep", true);
                Superstructure.getInstance().requestState(SuperstructureState.L3_PREP);
            }

            if(elapsedTime > 0.05 && l4PrepSafe() && (Superstructure.getInstance().getCurrentState() == SuperstructureState.PRESTAGE || Superstructure.getInstance().isReefPrepState()) && Superstructure.getInstance().getScoringFlag() == ScoringFlag.L4FLAG){
                Logger.getInstance().logEvent("Align to reef to L4 Prep", true);
                Superstructure.getInstance().requestState(SuperstructureState.L4_PREP);
            }
        }
        if (DriverStation.isAutonomousEnabled()) {
            if(elapsedTime > 0.05 && l4PrepSafe() && (Superstructure.getInstance().getCurrentState() == SuperstructureState.PRESTAGE || Superstructure.getInstance().isReefPrepState()) && Superstructure.getInstance().getScoringFlag() == ScoringFlag.L4FLAG){
                Logger.getInstance().logEvent("Align to reef to L4 Prep", true);
                Superstructure.getInstance().requestState(SuperstructureState.L4_PREP);
            }
        }

        // Auto Score Logic
        boolean autoScore = SmartDashboard.getBoolean("Align: Auto Score", true);

        if (Math.abs(rotationError) < rotationThreshold && depthAndLateralClose() && autoScore && (elapsedTime > 0.3 || depthAndLateralGood()) && Math.abs(drivetrain.getDrivetrainCurrentVelocity()) < 0.5) {
            Logger.getInstance().logEvent("Align to reef send to score", true);
            Superstructure.getInstance().sendToScore();
            SmartDashboard.putBoolean("Align: fire gamepiece", true);
            if(Superstructure.getInstance().isReefScoringState()){
                LimelightBack.getInstance().flashLED();
                SmartDashboard.putNumber("Align: Elapsed Time", elapsedTime);
            }
        }

        Logger.getInstance().logAlignToReef(lateralError, depthError, rotationError, lateralMagnitude, depthMagnitude, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Align: Elapsed Time", Timer.getFPGATimestamp() - initialTime);

        Superstructure.getInstance().sendToScore();
        if (desiredPose.isPresent())
            drivetrain.drive(new Translation2d(0,0), 0, false, null);
  
        Logger.getInstance().logEvent(
            "Reef " + commandName + " ended with errors: lateral " + lateralError + ", depth " + depthError + ", rotation " + rotationError,
            false
        );

        this.L4offset = 0;
        Superstructure.getInstance().setL4offset(0);
    }

    @Override
    public boolean isFinished() {
        if (desiredPose.isEmpty()) {
            Logger.getInstance().logEvent("Align to reef end reason: no desired pose", false);
            return true;
        }

        // if (Superstructure.getInstance().isReefScoringState())
        //     Logger.getInstance().logEvent("Align to reef is in scoring state", false);
        // if (Superstructure.getInstance().getRequestedState() == HP_INTAKE)
        //     Logger.getInstance().logEvent("Align to reef is in HP intake state", false);
        // if (Superstructure.getInstance().getCurrentState() == STOW)
        //     Logger.getInstance().logEvent("Align to reef is in stow state", false);

        return (
            Superstructure.getInstance().isReefScoringState() ||
            Superstructure.getInstance().getRequestedState() == HP_INTAKE ||
            Superstructure.getInstance().getCurrentState() == STOW
        ) && rotationGood() && depthAndLateralGood();
    }
}
