package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightClimber;
import frc.robot.utils.DriverOI;
import frc.robot.utils.DriverOI.DPadDirection;
import frc.robot.utils.Logger;

public class AlignToCage extends Command {
    private Drivetrain drivetrain;
    private LimelightClimber ll;

    private PIDController rotationController, yController;
    private double rotationP, rotationI, rotationD, rotationFF, rotationLowerP, rotationUseLowerPThreshold;
    private double yP, yI, yD, yFF;
    private double rotationThreshold, yThreshold;
    private double desiredAngle, desiredY;
    private double rotationError;

    public AlignToCage() {
        drivetrain = Drivetrain.getInstance();
        ll = LimelightClimber.getInstance();

        rotationP = 0.05;
        rotationI = 0.0;
        rotationD = 0.0;
        rotationFF = 0.0;
        rotationThreshold = 1.0;
        rotationLowerP = 0.03;
        rotationUseLowerPThreshold = 1.5;
        rotationController = new PIDController(rotationP, rotationI, rotationD);
        rotationController.enableContinuousInput(-180.0, 180.0);

        yP = 0.04;
        yI = 0.0;
        yD = 0.0;
        yFF = 0.0;
        yThreshold = 1;
        yController = new PIDController(yP, yI, yD);

        desiredAngle = 180.0;
        desiredY = 0.0;

        rotationError = 1000;

        SmartDashboard.putNumber("CageAlign: rotationP", rotationP);
        SmartDashboard.putNumber("CageAlign: rotationI", rotationI);
        SmartDashboard.putNumber("CageAlign: rotationD", rotationD);
        SmartDashboard.putNumber("CageAlign: rotationFF", rotationFF);
        SmartDashboard.putNumber("CageAlign: rotationThreshold", rotationThreshold);
        SmartDashboard.putNumber("CageAlign: rotationLowerP", rotationLowerP);
        SmartDashboard.putNumber("CageAlign: rotationUseLowerPThreshold", rotationUseLowerPThreshold);

        SmartDashboard.putNumber("CageAlign: yP", yP);
        SmartDashboard.putNumber("CageAlign: yI", yI);
        SmartDashboard.putNumber("CageAlign: yD", yD);
        SmartDashboard.putNumber("CageAlign: yFF", yFF);
        SmartDashboard.putNumber("CageAlign: yThreshold", yThreshold);

        SmartDashboard.putNumber("CageAlign: desiredAngle", desiredAngle);
        SmartDashboard.putNumber("CageAlign: desiredY", desiredY);
    }

    @Override
    public void initialize() {
        ll.setPipeline(0); 
        // drivetrain.setUseMegaTag(false);
        Logger.getInstance().logEvent("Align to Cage", true);
    }

    @Override
    public void execute() {
        rotationP = SmartDashboard.getNumber("CageAlign: rotationP", rotationP);
        rotationI = SmartDashboard.getNumber("CageAlign: rotationI", rotationI);
        rotationD = SmartDashboard.getNumber("CageAlign: rotationD", rotationD);
        rotationFF = SmartDashboard.getNumber("CageAlign: rotationFF", rotationFF);
        rotationThreshold = SmartDashboard.getNumber("CageAlign: rotationThreshold", rotationThreshold);
        rotationLowerP = SmartDashboard.getNumber("CageAlign: rotationLowerP", rotationLowerP);
        rotationUseLowerPThreshold = SmartDashboard.getNumber("CageAlign: rotationUseLowerPThreshold", rotationUseLowerPThreshold);

        yP = SmartDashboard.getNumber("CageAlign: yP", yP);
        yI = SmartDashboard.getNumber("CageAlign: yI", yI);
        yD = SmartDashboard.getNumber("CageAlign: yD", yD);
        yFF = SmartDashboard.getNumber("CageAlign: yFF", yFF);
        yThreshold = SmartDashboard.getNumber("CageAlign: yThreshold", yThreshold);

        desiredAngle = SmartDashboard.getNumber("CageAlign: desiredAngle", desiredAngle);
        desiredY = SmartDashboard.getNumber("CageAlign: desiredY", desiredY);
        
        yController.setPID(yP, yI, yD);
    
        if (Math.abs(rotationError) < rotationUseLowerPThreshold)
            rotationController.setP(rotationLowerP);
        else
            rotationController.setP(rotationP);
        rotationController.setI(rotationI);
        rotationController.setD(rotationD);

        rotationError = drivetrain.getHeading() - desiredAngle;
        double rotation = 0;
        if (Math.abs(rotationError) > rotationThreshold)
            rotation = rotationController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

        double yInput = 0;
        
        double ty = ll.getTy();
        if (Math.abs(ty) > yThreshold && ll.hasTarget())
            yInput = yController.calculate(ty) + Math.signum(ty) * yFF;
  
        double xDriverInput = DriverOI.getInstance().getForward();

        drivetrain.drive(new Translation2d(-xDriverInput, yInput), rotation, false, null);

        // Translation2d translation = DriverOI.getInstance().getSwerveTranslation();
        // if (DriverOI.getInstance().getDriverDPadInput() != DPadDirection.NONE) {
        //     translation = DriverOI.getInstance().getCardinalDirection();
        // }

        // drivetrain.drive(translation.times(-1), rotation, false, null);

        
        // Logger.getInstance().logAlignToCage(ty, yInput, rotationError, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        // drivetrain.setUseMegaTag(true);
        Logger.getInstance().logEvent("Align to Cage", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
