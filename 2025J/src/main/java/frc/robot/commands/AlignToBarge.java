// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.DriverOI.DPadDirection;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignToBarge extends Command {
    private Drivetrain drivetrain;

    private PIDController rotationPIDController;
    private double rotationUseLowerPThreshold, rotationThresholdP;
    private double desiredAngle, rotationThreshold;
    private double rotationP, rotationI, rotationD, rotationFF;
    private double rotationError;
    private double startTime;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AlignToBarge() {
        drivetrain = Drivetrain.getInstance();

        desiredAngle = 0;

        rotationP = 0.05;
        rotationI = 0.0;
        rotationD = 0.0;
        rotationFF = 0.0;
        rotationThresholdP = 0.03;
        rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        rotationPIDController.enableContinuousInput(-180.0, 180.0);

        rotationThreshold = 1;
        rotationUseLowerPThreshold = 1.5;
       
        addRequirements(drivetrain);
    }

    public boolean isInOwnSide() {
        boolean isInBlue = drivetrain.getPose().getX() < 17.55 / 2;

        boolean isPlayingBlue =
            DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        return isInBlue == isPlayingBlue;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        desiredAngle = isInOwnSide() ? FieldConstants.kBargeDesiredAngleOwn : FieldConstants.kBargeDesiredAngleOpponent;
        startTime = Timer.getFPGATimestamp();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        rotationError = drivetrain.getHeading() - desiredAngle;
    
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
        
        Translation2d translation = DriverOI.getInstance().getSwerveTranslation();

        if (DriverOI.getInstance().getDriverDPadInput() != DPadDirection.NONE) {
            translation = DriverOI.getInstance().getCardinalDirection();
        }
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
