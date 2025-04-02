
package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFrontLeft;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Constants.AlignmentConstants.HPAlign;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Logger;
import frc.robot.utils.MagnitudeCap;

public class AlignToProcessor extends Command {
    private Drivetrain drivetrain;
    private Superstructure superstructure;
    private Limelight[] cameras;

    private PIDController xPIDController, rotationPIDController;

    private double xP, xI, xD, xFF;
    private double rotationP, rotationI, rotationD, rotationFF;
    private double rotationLowerP, rotationUseLowerPThreshold;

    private double xThreshold, rotationThreshold;
    private double maxSpeed;

    private double desiredX, desiredAngle;
    private double xError, rotationError;

    private double sendToScoreLocation;

    public AlignToProcessor(double maxSpeed) {
        drivetrain = Drivetrain.getInstance();
        superstructure = Superstructure.getInstance();

        cameras = new Limelight[] {
            LimelightFrontLeft.getInstance(),
            LimelightFrontRight.getInstance(),
            LimelightBack.getInstance(),
        };

        xP = HPAlign.kLateralP;
        xI = HPAlign.kLateralI;
        xD = HPAlign.kLateralD;
        xFF = HPAlign.kLateralFF;
        
        rotationP = HPAlign.kRotationP;
        rotationI = HPAlign.kRotationI;
        rotationD = HPAlign.kRotationD;
        rotationFF = HPAlign.kRotationFF;
        rotationThreshold = HPAlign.kRotationThreshold;
        rotationLowerP = HPAlign.kRotationLowerP;
        rotationUseLowerPThreshold = HPAlign.kRotationUseLowerPThreshold;
        
        xPIDController = new PIDController(xP, xI, xD);
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        rotationPIDController.enableContinuousInput(-180.0, 180.0);

        xThreshold = HPAlign.kLateralThreshold;
        
        this.maxSpeed = maxSpeed;
        
        addRequirements(drivetrain);

        SmartDashboard.putNumber("ProcessorAlign: xP", xP);
        SmartDashboard.putNumber("ProcessorAlign: xI", xI);
        SmartDashboard.putNumber("ProcessorAlign: xD", xD);
        SmartDashboard.putNumber("ProcessorAlign: xFF", xFF);
        
        SmartDashboard.putNumber("ProcessorAlign: rotationP", rotationP);
        SmartDashboard.putNumber("ProcessorAlign: rotationI", rotationI);
        SmartDashboard.putNumber("ProcessorAlign: rotationD", rotationD);
        SmartDashboard.putNumber("ProcessorAlign: rotationFF", rotationFF);
        SmartDashboard.putNumber("ProcessorAlign: rotationThreshold", rotationThreshold);
        SmartDashboard.putNumber("ProcessorAlign: rotationLowerP", rotationLowerP);
        SmartDashboard.putNumber("ProcessorAlign: rotationUseLowerPThreshold", rotationUseLowerPThreshold);

        SmartDashboard.putNumber("ProcessorAlign: maxSpeed", maxSpeed);
        SmartDashboard.putNumber("ProcessorAlign: x threshold", 0.05);

        SmartDashboard.putNumber("ProcessorAlign: sendToScoreOffset", 0.65);
    }

    public boolean isInBlueSide() {
        return drivetrain.getPose().getX() < 17.55 / 2;
    }

    @Override
    public void initialize() {
        // int desiredTarget;
        // if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        //     desiredTarget = 16;
        //     desiredAngle = -90;
        // }
        // else {
        //     desiredTarget = 3;
        //     desiredAngle = 90;
        // }
        int desiredTarget = isInBlueSide() ? 16 : 3;
        desiredAngle = isInBlueSide() ? -90 : 90;

        Pose2d tagPose = Limelight.getAprilTagPose(desiredTarget);
        desiredX = tagPose.getX();
        
        double amount = 0.7; // SmartDashboard.getNumber("ProcessorAlign: sendToScoreOffset", 0.65);
        // if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        if (isInBlueSide())
            sendToScoreLocation = tagPose.getY() + amount;
        else
            sendToScoreLocation = tagPose.getY() - amount;

        SmartDashboard.putNumber("ProcessorAlign: desired tag", desiredTarget);
        SmartDashboard.putNumber("ProcessorAlign: desired pose X", desiredX);
        SmartDashboard.putNumber("ProcessorAlign: desired angle", desiredAngle);
        
        Logger.getInstance().logEvent("Align to Processor ID " + desiredTarget, true);

        xError = 10000;
        rotationError = 10000;
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

    @Override
    public void execute() {
        xThreshold = SmartDashboard.getNumber("ProcessorAlign: x threshold", 0.0);

        xP = SmartDashboard.getNumber("ProcessorAlign: xP", xP);
        xI = SmartDashboard.getNumber("ProcessorAlign: xI", xI);
        xD = SmartDashboard.getNumber("ProcessorAlign: xD", xD);
        xFF = SmartDashboard.getNumber("ProcessorAlign: xFF", xFF);

        rotationP = SmartDashboard.getNumber("ProcessorAlign: rotationP", rotationP);
        rotationI = SmartDashboard.getNumber("ProcessorAlign: rotationI", rotationI);
        rotationD = SmartDashboard.getNumber("ProcessorAlign: rotationD", rotationD);
        rotationFF = SmartDashboard.getNumber("ProcessorAlign: rotationFF", rotationFF);
        rotationThreshold = SmartDashboard.getNumber("ProcessorAlign: rotationThreshold", rotationThreshold);
        rotationLowerP = SmartDashboard.getNumber("ProcessorAlign: rotationLowerP", rotationLowerP);
        rotationUseLowerPThreshold = SmartDashboard.getNumber("ProcessorAlign: rotationUseLowerPThreshold", rotationUseLowerPThreshold);
        
        maxSpeed = SmartDashboard.getNumber("ProcessorAlign: maxSpeed", maxSpeed);
        
        rotationError = drivetrain.getHeadingBlue() - desiredAngle;

        xPIDController.setPID(xP, xI, xD);

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
        
        xError = estimatedPose.getX() - desiredX;

        double xInput = 0;
        if (Math.abs(xError) > xThreshold)
            xInput = xPIDController.calculate(xError) + Math.signum(xError) * xFF;

        SmartDashboard.putNumber("ProcessorAlign: xError", xError);
        SmartDashboard.putNumber("ProcessorAlign: y", estimatedPose.getY());
        SmartDashboard.putNumber("ProcessorAlign: send to score y", sendToScoreLocation);
        
        double strafe = DriverOI.getInstance().getStrafe() * DriveConstants.kMaxFloorSpeed;
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            strafe *= -1;
        
        Translation2d translation = new Translation2d(xInput, strafe);
        translation = MagnitudeCap.capMagnitude(translation, maxSpeed);
        
        if (superstructure.getCurrentState() == SuperstructureState.PROCESSOR_PREP) {
            Logger.getInstance().logEvent("Align to Processor score", true);
            if (isInBlueSide()) {
            // if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                if (estimatedPose.getY() <= sendToScoreLocation)
                    superstructure.sendToScore();
            }
            else {
                if (estimatedPose.getY() >= sendToScoreLocation)
                    superstructure.sendToScore();
            }
        }

        drivetrain.driveBlue(translation, rotation, true, null);
        Logger.getInstance().logAlignToProcessor(xError, rotationError, strafe, xInput, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.getInstance().logEvent("Align to Processor", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
