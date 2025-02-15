
package frc.robot.commands.ScoreCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PVFrontMiddle;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.Constants;

public class LeftAlignScore extends Command {
    private Drivetrain drivetrain;
    private PVFrontMiddle pvFrontMiddle;
    private PIDController translatePIDController, rotationPIDController;

    private double translateP, translateI, translateD, translateFF, translateThreshold, translateSetpoint;
    private double rotationP, rotationI, rotationD, rotationFF, rotationThreshold;
    private double rotationLowerP, rotationUseLowerPThreshold;

    private Pose2d desiredPose;
    private double tagBackMagnitude, tagLeftMagnitude;

    private double desiredAngle;

    public LeftAlignScore() {
        drivetrain = Drivetrain.getInstance();
        pvFrontMiddle = PVFrontMiddle.getInstance();

        translatePIDController = new PIDController(translateP, translateI, translateD);
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        
        translateP = 2;
        translateI = 0;
        translateD = 0;
        translateFF = 0;
        translateThreshold = 0.0254;
        translateSetpoint = 0;

        rotationP = 0.05;
        rotationI = 0;
        rotationD = 0;
        rotationFF = 0;
        rotationThreshold = 0.8;
        rotationLowerP = 0.03;
        rotationUseLowerPThreshold = 1.5;
        
        // center of robot distance to tag -- back (+ = back, - = forwards)
        tagBackMagnitude = 0.8;
        // center of robot distance to tag -- left (+ = left, - = right)
        tagLeftMagnitude = 0.1;

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
        }
    }

    @Override
    public void initialize() {
        // must see tag!

        int desiredTarget = (int) pvFrontMiddle.getTargetID();
        if (Constants.kReefDesiredAngle.containsKey(desiredTarget))
            desiredAngle = Constants.kReefDesiredAngle.get(desiredTarget);

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
        }

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
        
        if (!pvFrontMiddle.hasTarget()) {
            drivetrain.drive(new Translation2d(), rotation, false, null);
            return;
        }

        Pose2d estimatedPose = pvFrontMiddle.getEstimatedPose();

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
        drivetrain.drive(translation, rotation, true, null);

        //for L4: if it meets the translation threshold + a little bit, then were sending it to L4 PREP. AND we got command to L4 it.
    }

    @Override
    public void end(boolean interrupted) {
        Superstructure.getInstance().sendToScore();
    }

    @Override
    public boolean isFinished() {
         return (Math.abs(pvFrontMiddle.getEstimatedPose().getX() - desiredPose.getX()) < translateThreshold) 
                    && (Math.abs(pvFrontMiddle.getEstimatedPose().getY() - desiredPose.getY()) < translateThreshold)
                    && (Math.abs(desiredAngle + drivetrain.getHeading()) < rotationThreshold);

    }
}
