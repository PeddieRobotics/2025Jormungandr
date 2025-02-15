package frc.robot.commands.ReefCommands;

import org.ejml.data.DEigenpair;
import org.opencv.photo.Photo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PVFrontLeft;
import frc.robot.subsystems.PVFrontMiddle;
import frc.robot.subsystems.PVFrontRight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.utils.Constants;

public class AlignToReefEstimatedPose extends Command {
    private Drivetrain drivetrain;
    // private PhotonVision[] cameras;
    private PhotonVision camera;

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
        // cameras = new PhotonVision[] {
        //     PVFrontLeft.getInstance(),
        //     PVFrontMiddle.getInstance(),
        //     PVFrontRight.getInstance(),
        // };
        camera = PVFrontRight.getInstance();

        translatePIDController = new PIDController(translateP, translateI, translateD);
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        
        translateP = 2.0;
        translateI = 0;
        translateD = 0;
        translateFF = 0;
        translateThreshold = 0.015;
        translateSetpoint = 0;

        rotationP = 0.04;
        rotationI = 0;
        rotationD = 0;
        rotationFF = 0;
        rotationThreshold = 1;
        rotationLowerP = 0.03;
        rotationUseLowerPThreshold = 1.5;
        
        // center of robot distance to tag -- back (+ = back, - = forwards)
        tagBackMagnitude = 0.3;
        // center of robot distance to tag -- left (+ = left, - = right)
        tagLeftMagnitude = 0.1651;
        
        maxSpeed = 2.0;

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
        // must see tag!
        PhotonVision pvFrontMiddle = PVFrontMiddle.getInstance();
        if (!pvFrontMiddle.hasTarget()) {
            desiredAngle = 0;
            desiredPose = null;
            return;
        }

        int desiredTarget = (int) pvFrontMiddle.getTargetID();
        if (Constants.kReefDesiredAngle.containsKey(desiredTarget))
            desiredAngle = Constants.kReefDesiredAngle.get(desiredTarget) + 180;
        
        Pose2d tagPose = pvFrontMiddle.getAprilTagPose();
        double tagAngle = tagPose.getRotation().getRadians();

        tagBackMagnitude = SmartDashboard.getNumber("align tagBackMagnitude", tagBackMagnitude);
        tagLeftMagnitude = SmartDashboard.getNumber("align tagLeftMagnitude", tagLeftMagnitude);

        // move the apriltag "forward"
        desiredPose = new Pose2d(
            tagPose.getX() + tagBackMagnitude * Math.cos(tagAngle) + tagLeftMagnitude * Math.sin(tagAngle),
            tagPose.getY() + tagBackMagnitude * Math.sin(tagAngle) - tagLeftMagnitude * Math.cos(tagAngle),
            new Rotation2d(0)
        );
    }
    
    // TODO: find camera with lowest reprojection error
    private Pose2d getBestEstimatedPose() {
        return new Pose2d();
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

        double rotationError = desiredAngle + drivetrain.getHeading();

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
        
        // if (!pvFrontMiddle.hasTarget()) {
        //     drivetrain.drive(new Translation2d(), rotation, false, null);
        //     return;
        // }

        Pose2d estimatedPose = camera.hasTarget() ? camera.getEstimatedPose() : drivetrain.getPose();

        double xError = estimatedPose.getX() - desiredPose.getX();
        double yError = estimatedPose.getY() - desiredPose.getY();
        
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
