package frc.robot.commands.ReefCommands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFrontLeft;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.CalculateReefTarget;
import frc.robot.utils.Constants;

public class AlignToReefEstimatedPose extends Command {
    private Drivetrain drivetrain;
    private Limelight[] cameras;

    private PIDController translatePIDController, rotationPIDController;

    private double translateP, translateI, translateD, translateFF, translateThreshold, translateSetpoint;
    private double rotationP, rotationI, rotationD, rotationFF, rotationThreshold;
    private double rotationLowerP, rotationUseLowerPThreshold;
    private double maxSpeed;

    private Pose2d desiredPose;
    private double tagBackMagnitude, tagLeftMagnitude;

    private double desiredAngle;

    public AlignToReefEstimatedPose() {
        drivetrain = Drivetrain.getInstance();
        cameras = new Limelight[] {
            LimelightFrontMiddle.getInstance(),
            LimelightFrontLeft.getInstance(),
            LimelightFrontRight.getInstance(),
        };

        translatePIDController = new PIDController(translateP, translateI, translateD);
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        
        translateP = 2.3;
        translateI = 0;
        translateD = 0;
        translateFF = 0;
        translateThreshold = 0.015;
        translateSetpoint = 0;

        rotationP = 0.05;
        rotationI = 0;
        rotationD = 0;
        rotationFF = 0;
        rotationThreshold = 0.5;
        rotationLowerP = 0.03;
        rotationUseLowerPThreshold = 1.5;
        
        // center of robot distance to tag -- back (+ = back, - = forwards)
        tagBackMagnitude = 0.5;
        // center of robot distance to tag -- left (+ = left, - = right)
        tagLeftMagnitude = 0.1651;
        
        maxSpeed = 3.0;

        addRequirements(drivetrain);
        
        {
            SmartDashboard.putNumber("align translateP", translateP);
            SmartDashboard.putNumber("align translateI", translateI);
            SmartDashboard.putNumber("align translateD", translateD);
            SmartDashboard.putNumber("align translateFF", translateFF);
            SmartDashboard.putNumber("align translateThreshold", translateThreshold);
            SmartDashboard.putNumber("align translateSetpoint", translateSetpoint);
            
            SmartDashboard.putNumber("align rotationP", rotationP);
            SmartDashboard.putNumber("align rotationI", rotationI);
            SmartDashboard.putNumber("align rotationD", rotationD);
            SmartDashboard.putNumber("align rotationFF", rotationFF);
            SmartDashboard.putNumber("align rotationThreshold", rotationThreshold);
            SmartDashboard.putNumber("align rotationLowerP", rotationLowerP);
            SmartDashboard.putNumber("align rotationUseLowerPThreshold", rotationUseLowerPThreshold);

            SmartDashboard.putNumber("align tagBackMagnitude", tagBackMagnitude);
            SmartDashboard.putNumber("align tagLeftMagnitude", tagLeftMagnitude);

            SmartDashboard.putNumber("align maxSpeed", maxSpeed);
        }
    }

    @Override
    public void initialize() {
        // TODO: restore
        // int desiredTarget = CalculateReefTarget.calculateTargetID();
        int desiredTarget = (int) LimelightFrontMiddle.getInstance().getTargetID();

        if (Constants.kReefDesiredAngle.containsKey(desiredTarget))
            desiredAngle = Constants.kReefDesiredAngle.get(desiredTarget);
        
        Pose2d tagPose = Limelight.getAprilTagPose(desiredTarget);
        double tagAngle = tagPose.getRotation().getRadians();

        tagBackMagnitude = SmartDashboard.getNumber("align tagBackMagnitude", tagBackMagnitude);
        tagLeftMagnitude = SmartDashboard.getNumber("align tagLeftMagnitude", tagLeftMagnitude);

        // move the apriltag "forward"
        desiredPose = new Pose2d(
            tagPose.getX() + tagBackMagnitude * Math.cos(tagAngle) + tagLeftMagnitude * Math.sin(tagAngle),
            tagPose.getY() + tagBackMagnitude * Math.sin(tagAngle) - tagLeftMagnitude * Math.cos(tagAngle),
            new Rotation2d(0)
        );
        
        SmartDashboard.putNumber("align desired tag", desiredTarget);
    }
    
    private Optional<Pose2d> getBestEstimatedPose() {
        // double bestReprojErr = Integer.MAX_VALUE;
        // Pose2d bestPose = drivetrain.getPose();
        // for (PhotonVision camera : cameras) {
        //     if (!camera.hasTarget())
        //         continue;
        //     if (camera.getReprojectionError() < bestReprojErr) {
        //         bestReprojErr = camera.getReprojectionError();
        //         bestPose = camera.getEstimatedPose();
        //     }
        // }


        for (Limelight camera : cameras) {
            Optional<Pose2d> measurement = camera.getEstimatedPoseMT2();
            if (measurement.isPresent() && camera.hasTarget())
                return Optional.of(measurement.get());
        }

        return Optional.empty();
    }

    @Override
    public void execute() {
        {
            translateP = SmartDashboard.getNumber("align translateP", translateP);
            translateI = SmartDashboard.getNumber("align translateI", translateI);
            translateD = SmartDashboard.getNumber("align translateD", translateD);
            translateFF = SmartDashboard.getNumber("align translateFF", translateFF);
            translateThreshold = SmartDashboard.getNumber("align translateThreshold", translateThreshold);
            translateSetpoint = SmartDashboard.getNumber("align translateSetpoint", translateSetpoint);

            rotationP = SmartDashboard.getNumber("align rotationP", rotationP);
            rotationI = SmartDashboard.getNumber("align rotationI", rotationI);
            rotationD = SmartDashboard.getNumber("align rotationD", rotationD);
            rotationFF = SmartDashboard.getNumber("align rotationFF", rotationFF);
            rotationThreshold = SmartDashboard.getNumber("align rotationThreshold", rotationThreshold);
            rotationLowerP = SmartDashboard.getNumber("align rotationLowerP", rotationLowerP);
            rotationUseLowerPThreshold = SmartDashboard.getNumber("align rotationUseLowerPThreshold", rotationUseLowerPThreshold);

            maxSpeed = SmartDashboard.getNumber("align maxSpeed", maxSpeed);
        }
        
        if (desiredPose == null)
            return;

        double rotationError = drivetrain.getHeading() - desiredAngle;

        translatePIDController.setPID(translateP, translateI, translateD);
        if(Math.abs(rotationError) < rotationUseLowerPThreshold)
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
            // TODO
            drivetrain.drive(new Translation2d(0, 0), 0, true, null);
            return;
        }
        Pose2d estimatedPose = estimatedPoseOptional.get();

        double xError = desiredPose.getX() - estimatedPose.getX();
        double yError = desiredPose.getY() - estimatedPose.getY();
        
        xError *= -1;
        yError *= -1;

        SmartDashboard.putNumber("align xError", xError);
        SmartDashboard.putNumber("align yError", yError);
        
        double xTranslate = 0, yTranslate = 0;
        if (Math.abs(xError) > translateThreshold)
            xTranslate = translatePIDController.calculate(xError) + Math.signum(xError) * translateFF;
        if (Math.abs(yError) > translateThreshold)
            yTranslate = translatePIDController.calculate(yError) + Math.signum(yError) * translateFF;

        Translation2d translation = new Translation2d(xTranslate, yTranslate);
        double translateX = translation.getX();
        double translateY = translation.getY();
        double translateX_sgn = Math.signum(translateX);
        double translateY_sgn = Math.signum(translateY);
        double desaturatedX = Math.min(Math.abs(translateX), maxSpeed);
        double desaturatedY = Math.min(Math.abs(translateY), maxSpeed);
        translation = new Translation2d(translateX_sgn * desaturatedX, translateY_sgn * desaturatedY);

        drivetrain.drive(translation, rotation, true, null);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return desiredPose == null;
    }
}
