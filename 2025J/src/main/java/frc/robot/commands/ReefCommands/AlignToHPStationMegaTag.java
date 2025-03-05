// TODO: Use pythagorean theorem thingy

package frc.robot.commands.ReefCommands;

import static frc.robot.subsystems.Superstructure.SuperstructureState.HP_INTAKE;

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
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.CalculateReefTarget;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import frc.robot.utils.Constants.AlignmentConstants.ReefAlignEstimatedPose;

public class AlignToHPStationMegaTag extends Command {
    private Drivetrain drivetrain;

    private PIDController translatePIDController, rotationPIDController;

    private double translateP, translateI, translateD, translateFF, translateThreshold;
    private double rotationP, rotationI, rotationD, rotationFF, rotationThreshold;
    private double rotationLowerP, rotationUseLowerPThreshold;
    private double maxSpeed;

    private Pose2d desiredPose;
    private double desiredAngle;

    public AlignToHPStationMegaTag() {
        drivetrain = Drivetrain.getInstance();
        
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
                
        maxSpeed = ReefAlignEstimatedPose.kMaxSpeed;
        
        addRequirements(drivetrain);
        
        SmartDashboard.putNumber("align translateP", translateP);
        SmartDashboard.putNumber("align translateI", translateI);
        SmartDashboard.putNumber("align translateD", translateD);
        SmartDashboard.putNumber("align translateFF", translateFF);
        SmartDashboard.putNumber("align translateThreshold", translateThreshold);
        
        SmartDashboard.putNumber("align rotationP", rotationP);
        SmartDashboard.putNumber("align rotationI", rotationI);
        SmartDashboard.putNumber("align rotationD", rotationD);
        SmartDashboard.putNumber("align rotationFF", rotationFF);
        SmartDashboard.putNumber("align rotationThreshold", rotationThreshold);
        SmartDashboard.putNumber("align rotationLowerP", rotationLowerP);
        SmartDashboard.putNumber("align rotationUseLowerPThreshold", rotationUseLowerPThreshold);

        SmartDashboard.putNumber("align maxSpeed", maxSpeed);
    }

    @Override
    public void initialize() {
        desiredAngle = 46.5;
        desiredPose = new Pose2d(0.87, 1.1, new Rotation2d(Math.toRadians(desiredAngle)));
        Superstructure.getInstance().requestState(HP_INTAKE);
    }

    @Override
    public void execute() {
        {
            translateP = SmartDashboard.getNumber("align translateP", translateP);
            translateI = SmartDashboard.getNumber("align translateI", translateI);
            translateD = SmartDashboard.getNumber("align translateD", translateD);
            translateFF = SmartDashboard.getNumber("align translateFF", translateFF);
            translateThreshold = SmartDashboard.getNumber("align translateThreshold", translateThreshold);

            rotationP = SmartDashboard.getNumber("align rotationP", rotationP);
            rotationI = SmartDashboard.getNumber("align rotationI", rotationI);
            rotationD = SmartDashboard.getNumber("align rotationD", rotationD);
            rotationFF = SmartDashboard.getNumber("align rotationFF", rotationFF);
            rotationThreshold = SmartDashboard.getNumber("align rotationThreshold", rotationThreshold);
            rotationLowerP = SmartDashboard.getNumber("align rotationLowerP", rotationLowerP);
            rotationUseLowerPThreshold = SmartDashboard.getNumber("align rotationUseLowerPThreshold", rotationUseLowerPThreshold);

            maxSpeed = SmartDashboard.getNumber("align maxSpeed", maxSpeed);
        }
        

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

        Pose2d estimatedPose = Drivetrain.getInstance().getPose();

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
        drivetrain.drive(new Translation2d(0,0), 0, false, null);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
