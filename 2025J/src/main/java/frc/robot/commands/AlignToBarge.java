// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightClimber;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.DriverOI.DPadDirection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignToBarge extends Command {
    private Drivetrain drivetrain;
    private LimelightClimber limelight;

    private PIDController rotationPIDController, translationPIDController;
    private double rotationUseLowerPThreshold, rotationThresholdP;
    private double desiredAngle, rotationThreshold;
    private double translationThreshold;
    private double rotationP, rotationI, rotationD, rotationFF;
    private double translationP, translationI, translationD, translationFF;
    private double rotationError, translationError;
    private double startTime;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AlignToBarge() {
        drivetrain = Drivetrain.getInstance();
        limelight = LimelightClimber.getInstance();

        desiredAngle = 0;

        rotationP = 0.05;
        rotationI = 0.0;
        rotationD = 0.0;
        rotationFF = 0.0;
        rotationThresholdP = 0.03;
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        
        translationP = 0.0;
        translationI = 0.0;
        translationD = 0.0;
        translationFF = 0.0;
        translationPIDController = new PIDController(translationP, translationI, translationD);

        rotationThreshold = 1;
        rotationUseLowerPThreshold = 1.5;
        
        translationThreshold = 1;
        
        SmartDashboard.putNumber("BargeAlign: translationP", translationP);
        SmartDashboard.putNumber("BargeAlign: translationI", translationI);
        SmartDashboard.putNumber("BargeAlign: translationD", translationD);
        SmartDashboard.putNumber("BargeAlign: translationFF", translationFF);
        SmartDashboard.putNumber("BargeAlign: translationThreshold", translationThreshold);
       
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        desiredAngle = FieldConstants.kCageDesiredAngle;
        startTime = Timer.getFPGATimestamp();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        translationP = SmartDashboard.getNumber("BargeAlign: translationP", translationP);
        translationI = SmartDashboard.getNumber("BargeAlign: translationI", translationI);
        translationD = SmartDashboard.getNumber("BargeAlign: translationD", translationD);
        translationFF = SmartDashboard.getNumber("BargeAlign: translationFF", translationFF);
        translationThreshold = SmartDashboard.getNumber("BargeAlign: translationThreshold", translationThreshold);
        translationPIDController.setPID(translationP, translationI, translationD);

        rotationError = desiredAngle + drivetrain.getHeading();
    
        // set rotation PID controller
        if(Math.abs(rotationError) < rotationUseLowerPThreshold)
            rotationPIDController.setP(rotationThresholdP);
        else
            rotationPIDController.setP(rotationP);
        rotationPIDController.setI(rotationI);
        rotationPIDController.setD(rotationD);
        
        double rotation = 0;
        if (Math.abs(rotationError) > rotationThreshold)
            rotation = rotationPIDController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

        double tx = limelight.getTxAverage(), y = 0;;
        if (Math.abs(tx) > translationThreshold)
            y = translationPIDController.calculate(tx) + Math.signum(tx) * translationFF;
        
        // Translation2d translation = DriverOI.getInstance().getSwerveTranslation();
        Translation2d translation = new Translation2d(DriverOI.getInstance().getForward(), y);

        // if (DriverOI.getInstance().getDriverDPadInput() != DPadDirection.NONE)
        //     translation = DriverOI.getInstance().getCardinalDirection();

        drivetrain.drive(translation, rotation, false, null);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        double elapsedTime = Timer.getFPGATimestamp() - SmartDashboard.getNumber("Forward start", startTime);
        SmartDashboard.putNumber("time elapsed since start", elapsedTime);

        drivetrain.drive(new Translation2d(0,0), 0, false, null);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
