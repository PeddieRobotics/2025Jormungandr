package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.utils.DriverOI;

public class AlignToCage extends Command {
    private Drivetrain drivetrain;
    private LimelightFrontMiddle ll;

    private PIDController rotationController, yController, lockedController;
    private double rotationP, rotationI, rotationD, rotationFF, rotationLowerP, rotationUseLowerPThreshold;
    private double yP, yI, yD, yFF;
    private double lockedP, lockedI, lockedD, lockedFF;
    private double rotationThreshold, yThreshold, lockedThreshold;
    private double desiredAngle, desiredY;
    private double rotationError;

    private double startTime;
    private double minOdometryY, maxOdometryY;

    public AlignToCage() {
        drivetrain = Drivetrain.getInstance();
        ll = LimelightFrontMiddle.getInstance();

        rotationP = 0.08;
        rotationI = 0.0;
        rotationD = 0.0;
        rotationFF = 0.0;
        rotationThreshold = 0.5;
        rotationLowerP = 0.06;
        rotationUseLowerPThreshold = 1.5;
        rotationController = new PIDController(rotationP, rotationI, rotationD);
        rotationController.enableContinuousInput(-180.0, 180.0);

        yP = 0.04;
        yI = 0.0;
        yD = 0.0;
        yFF = 0.0;
        yThreshold = 1;
        yController = new PIDController(yP, yI, yD);

        lockedP = 0.05;
        lockedI = 0.0;
        lockedD = 0.0;
        lockedFF = 0.0;
        lockedThreshold = 0.01;
        lockedController = new PIDController(lockedP, lockedI, lockedD);

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

        SmartDashboard.putNumber("CageAlign: lockedP", lockedP);
        SmartDashboard.putNumber("CageAlign: lockedI", lockedI);
        SmartDashboard.putNumber("CageAlign: lockedD", lockedD);
        SmartDashboard.putNumber("CageAlign: lockedFF", lockedFF);
        SmartDashboard.putNumber("CageAlign: lockedThreshold", lockedThreshold);

        SmartDashboard.putNumber("CageAlign: desiredAngle", desiredAngle);
        SmartDashboard.putNumber("CageAlign: desiredY", desiredY);
    }

    @Override
    public void initialize() {
        startTime = 0;
        ll.setPipeline(1); 

        minOdometryY = 10000;
        maxOdometryY = -10000;
        
        drivetrain.setUseMegaTag(false);
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

        lockedP = SmartDashboard.getNumber("CageAlign: lockedP", lockedP);
        lockedI = SmartDashboard.getNumber("CageAlign: lockedI", lockedI);
        lockedD = SmartDashboard.getNumber("CageAlign: lockedD", lockedD);
        lockedFF = SmartDashboard.getNumber("CageAlign: lockedFF", lockedFF);
        lockedThreshold = SmartDashboard.getNumber("CageAlign: lockedThreshold", lockedThreshold);

        desiredAngle = SmartDashboard.getNumber("CageAlign: desiredAngle", desiredAngle);
        desiredY = SmartDashboard.getNumber("CageAlign: desiredY", desiredY);
        
        yController.setPID(yP, yI, yD);
        lockedController.setPID(lockedP, lockedI, lockedD);
    
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

        double currentTime = Timer.getFPGATimestamp();
        double yInput = 0;
        
        double odometryY = drivetrain.getPose().getTranslation().getY();
        SmartDashboard.putNumber("CageAlign: minOdometryY", minOdometryY);
        SmartDashboard.putNumber("CageAlign: maxOdometryY", maxOdometryY);
        SmartDashboard.putNumber("CageAlign: odometryY", odometryY);
        SmartDashboard.putNumber("CageAlign: startTime", startTime);
        SmartDashboard.putNumber("CageAlign: currentTime", currentTime);

        double tx = ll.getTxAverage();
        if (Math.abs(tx) > yThreshold && ll.hasTarget())
            yInput = yController.calculate(tx) + Math.signum(tx) * yFF;

        // if (DriverOI.getInstance().getLeftBumperHeld())
        //     yInput = 0;
  
        double xDriverInput = DriverOI.getInstance().getForward();
        drivetrain.drive(new Translation2d(xDriverInput, yInput), rotation, false, null);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setUseMegaTag(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}