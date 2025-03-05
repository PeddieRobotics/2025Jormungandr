// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ReefCommands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFrontLeft;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.utils.CalculateReefTarget;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AlignmentConstants;
import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToReef2D extends Command {
    private Drivetrain drivetrain;
    private Limelight limelight;

    private PIDController rotationPidController, translationPidController, distancePidController;
    private double rotationUseLowerPThreshold, rotationThresholdP;
    private double translationThreshold, rotationThreshold, distanceThreshold;
    private double desiredAngle;
    private double translationP, translationI, translationD, translationFF;
    private double rotationP, rotationI, rotationD, rotationFF;
    private double distanceP, distanceI, distanceD, distanceFF;
    private double desiredDistance;
    private double txValue, rotationError, distanceError;
    private double startTime;
    private double desiredTranslation;
    private Translation2d translation;
    private boolean shouldEnd;

    public AlignToReef2D(AlignmentDestination destination) {
        drivetrain = Drivetrain.getInstance();
        
        switch (destination) {
            case LEFT:
                limelight = LimelightFrontRight.getInstance();
                break;
            case MIDDLE:
                limelight = LimelightFrontMiddle.getInstance();
                break;
            case RIGHT:
                limelight = LimelightFrontLeft.getInstance();
                break;
        }

        desiredAngle = 0;
        desiredDistance = 0.49;
        desiredTranslation = 0.0;

        SmartDashboard.putNumber("Desired Distance", desiredDistance);
        SmartDashboard.putNumber("Desired Translation", desiredTranslation);

        distanceP = 2;
        distanceI = 0;
        distanceD = 0;
        // distanceD = 0.0762;
        distanceFF = 0;
        distancePidController = new PIDController(distanceP, distanceI , distanceD);

        SmartDashboard.putNumber("Distance P", distanceP);
        SmartDashboard.putNumber("Distance I", distanceI);
        SmartDashboard.putNumber("Distance D", distanceD);
        SmartDashboard.putNumber("Distance FF", distanceFF);
        
        translationP = 1.75;
        translationI = 0;
        translationD = 0;
        translationFF = 0.0;
        translationPidController = new PIDController(translationP, translationI , translationD);

        SmartDashboard.putNumber("PhilipAlign P", translationP);
        SmartDashboard.putNumber("PhilipAlign I", translationI);
        SmartDashboard.putNumber("PhilipAlign D", translationD);
        SmartDashboard.putNumber("PhilipAlign FF", translationFF);

        rotationP = 0.09;
        rotationI = 0.0;
        rotationD = 0.0;
        rotationFF = 0.0;
        rotationThresholdP = 0.07;
        rotationPidController = new PIDController(rotationP, rotationI, rotationD);

        SmartDashboard.putNumber("Rotation P", rotationP);
        SmartDashboard.putNumber("Rotation Threshold P", rotationThresholdP);
        SmartDashboard.putNumber("Rotation I", rotationI);
        SmartDashboard.putNumber("Rotation D", rotationD);
        SmartDashboard.putNumber("Rotation FF", rotationFF);

        distanceThreshold = 0.0254;
        rotationThreshold = 0.5;
        translationThreshold = 0.0254;
        rotationUseLowerPThreshold = 1.5;
        
        SmartDashboard.putNumber("rotationThreshold", rotationThreshold);
        SmartDashboard.putNumber("translationThreshold", translationThreshold);
        SmartDashboard.putNumber("distanceThreshold", distanceThreshold);
        SmartDashboard.putNumber("rotationUseLowerPThreshold", rotationUseLowerPThreshold);

        translation = new Translation2d(0, 0);
        
        shouldEnd = false;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Limelight middle = LimelightFrontMiddle.getInstance();
        // TODO: temporary as middle limelight got smashed by L1 coral
        // Limelight middle = limelight;

        if (!middle.hasTarget()){
            shouldEnd = true;
            return;
        }

        int desiredTarget;
        if (middle.getNumberOfTagsSeen() == 1)
            desiredTarget = (int) middle.getTargetID();
        else 
            desiredTarget = CalculateReefTarget.calculateTargetID();

        if (!AlignmentConstants.kReefDesiredAngle.containsKey(desiredTarget)){
            shouldEnd = true;
            return;
        }
        
        limelight.setPriorityTag(desiredTarget);
        desiredAngle = AlignmentConstants.kReefDesiredAngle.get(desiredTarget);

        translation = new Translation2d(0, 0);
        shouldEnd = false;
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (shouldEnd)
            return;

        translationP = SmartDashboard.getNumber("PhilipAlign P", translationP);
        translationI = SmartDashboard.getNumber("PhilipAlign I", translationI);
        translationD = SmartDashboard.getNumber("PhilipAlign D", translationD);
        translationFF = SmartDashboard.getNumber("PhilipAlign FF", translationFF);

        rotationP = SmartDashboard.getNumber("Rotation P", rotationP);
        rotationThresholdP = SmartDashboard.getNumber("Rotation Threshold P", rotationThresholdP);
        rotationI = SmartDashboard.getNumber("Rotation I", rotationI);
        rotationD = SmartDashboard.getNumber("Rotation D", rotationD);
        rotationFF = SmartDashboard.getNumber("Rotation FF", rotationFF);

        distanceP = SmartDashboard.getNumber("Distance P", distanceP);
        distanceI = SmartDashboard.getNumber("Distance I", distanceI);
        distanceD = SmartDashboard.getNumber("Distance D", distanceD);
        distanceFF = SmartDashboard.getNumber("Distance FF", distanceFF);

        rotationThreshold = SmartDashboard.getNumber("rotationThreshold", rotationThreshold);
        translationThreshold = SmartDashboard.getNumber("translationThreshold", translationThreshold);
        distanceThreshold = SmartDashboard.getNumber("distanceThreshold", distanceThreshold);
        rotationUseLowerPThreshold = SmartDashboard.getNumber("rotationUseLowerPThreshold", rotationUseLowerPThreshold);

        desiredDistance = SmartDashboard.getNumber("Desired Distance", desiredDistance);
        desiredTranslation = SmartDashboard.getNumber("Desired Translation", desiredTranslation);

        rotationError = drivetrain.getHeading() - desiredAngle;
        double desiredTx = drivetrain.getHeading() - desiredAngle; // = gyro - desiredAngle
        
        SmartDashboard.putNumber("desired Tx", desiredTx);

        // set rotation PID controller
        if(Math.abs(rotationError) < rotationUseLowerPThreshold)
            rotationPidController.setP(rotationThresholdP);
        else
            rotationPidController.setP(rotationP);
        rotationPidController.setI(rotationI);
        rotationPidController.setD(rotationD);

        // set translation PID controller
        translationPidController.setPID(translationP, translationI, translationD);
        distancePidController.setPID(distanceP, distanceI, distanceD);

        double distance = limelight.getDistanceEstimatedPose();
        
        double horizontalTranslation = 0;
        double forBackTranslation = 0;
        if (limelight.hasTarget()) {
            txValue = limelight.getTx();
            
            double translationError = distance * Math.sin((txValue - desiredTx) * (Math.PI/180));
            
            SmartDashboard.putNumber("Translation Error", translationError);
            SmartDashboard.putNumber("Distance Error", distanceError);
            distanceError = distance - desiredDistance;

            if (Math.abs(translationError) > translationThreshold)
                horizontalTranslation = translationPidController.calculate(translationError) + Math.signum(translationError) * translationFF;

            if (Math.abs(distanceError) > distanceThreshold)
                forBackTranslation = distancePidController.calculate(distanceError) - Math.signum(distanceError) * distanceFF;

            translation = new Translation2d(-forBackTranslation, horizontalTranslation);

            
        }
        else
            translation = new Translation2d(translation.getX() / 2, translation.getY() / 2);

        SmartDashboard.putNumber("Rotation Error", rotationError);

        // calculate rotation
        double rotation = 0;
        if (Math.abs(rotationError) > rotationThreshold)
            rotation = rotationPidController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;
        
        double translateX = translation.getX();
        double translateY = translation.getY();
        double translateX_sgn = Math.signum(translateX);
        double translateY_sgn = Math.signum(translateY);
        double desaturatedX = Math.min(Math.abs(translateX), 1);
        double desaturatedY = Math.min(Math.abs(translateY), 1);
        translation = new Translation2d(translateX_sgn * desaturatedX, translateY_sgn * desaturatedY);

        drivetrain.drive(translation, rotation, false, null);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // double elapsedTime = Timer.getFPGATimestamp() - SmartDashboard.getNumber("Forward start", startTime);
        // SmartDashboard.putNumber("time elapsed since start", elapsedTime);

        limelight.setPriorityTag(0);

        drivetrain.drive(new Translation2d(0,0), 0, false, null);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return shouldEnd;
    }
}
