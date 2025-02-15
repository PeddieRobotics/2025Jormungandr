// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PVFrontMiddle;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.DriverOI;

import com.ctre.phoenix6.signals.PIDRefPIDErr_ClosedLoopModeValue;
import com.fasterxml.jackson.databind.DeserializationFeature;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignToCage extends Command {
  private Drivetrain drivetrain;
  private DriverOI oi;
  //TODO: not right camera
  private PVFrontMiddle shooterCam;

  private PIDController rotationPidController, translationPidController, distancePidController;
  private double rotationUseLowerPThreshold, rotationThresholdP;
  private double translationThreshold, rotationThreshold;
  private double desiredAngle;
  private double translationP, translationI, translationD, translationFF;
  private double rotationP, rotationI, rotationD, rotationFF;
  private double txValue, rotationError, distanceError;
  private double startTime;
  private double desiredTranslation;
  private Translation2d translation;
  private boolean isAuto;

  // private Logger logger;
  
  //private double constantSpeedAuto;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToCage(boolean isAuto) {
    this.isAuto = isAuto;

    drivetrain = Drivetrain.getInstance();
    shooterCam = PVFrontMiddle.getInstance();
    //logger = Logger.getInstance();

    desiredAngle = 0;
    desiredTranslation = 0.07;
    
    translationP = 0.25;
    translationI = 0;
    translationD = 0;
    translationFF = 0.001;
    translationPidController = new PIDController(translationP, translationI , translationD);

    rotationP = 0.1;
    rotationI = 0.0;
    rotationD = 0.0;
    rotationFF = 0.0;
    rotationThresholdP = 0.04;
    rotationPidController = new PIDController(rotationP, rotationI, rotationD);

    rotationThreshold = 1;
    translationThreshold = 0.0254;
    rotationUseLowerPThreshold = 1.5;
  

    translation = new Translation2d(0, 0);

    //constantSpeedAuto = 0.5;

    //SmartDashboard.putNumber("Constant Speed Auto", constantSpeedAuto);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oi = DriverOI.getInstance();

    desiredAngle = FieldConstants.kCageDesiredAngle;

    translation = new Translation2d(0, 0);

    startTime = Timer.getFPGATimestamp();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterCam.setPipeline(drivetrain.getPipelineNumber());
    double distanceFromCage = shooterCam.getFilteredDistanceTy();

    //TODO: getHeading is in clockwise positive, should be counterclockwise on 2025j
    rotationError = desiredAngle + drivetrain.getHeading();
    double desiredTx = drivetrain.getHeading() - desiredAngle; // = gyro - desiredAngle
  
    // set rotation PID controller
    if(Math.abs(rotationError) < rotationUseLowerPThreshold)
      rotationPidController.setP(rotationThresholdP);
    else
      rotationPidController.setP(rotationP);
    rotationPidController.setI(rotationI);
    rotationPidController.setD(rotationD);

    // set translation PID controller
    translationPidController.setPID(translationP, translationI, translationD);
    
    double horizontalTranslation = 0;
    double forBackTranslation = oi.getForward();
    if (shooterCam.hasTarget()) {
      txValue = shooterCam.getTx();
      
      double translationError =  txValue - desiredTx;
      
      SmartDashboard.putNumber("Translation Error", translationError);

      if (Math.abs(translationError) > translationThreshold)
        horizontalTranslation = translationPidController.calculate(translationError) + Math.signum(translationError) * translationFF;

      translation = new Translation2d(forBackTranslation, horizontalTranslation/5);

      //logger.cmdTranslationEntry.append(translationError);
    }

    SmartDashboard.putNumber("Rotation Error", rotationError);
    //logger.cmdRotationEntry.append(rotationError);

    // calculate rotation
    double rotation = 0;
    if (Math.abs(rotationError) > rotationThreshold)
      rotation = rotationPidController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

    // calculate forward-backward translation (a dot b)
    // double b1 = -oi.getStrafe();
    // double b2 = oi.getForward();
    // double forBackTranslation = (a1*b1 + a2*b2) * Constants.DriveConstants.kMaxFloorSpeed;

    //constantSpeedAuto = SmartDashboard.getNumber("Constant Speed Auto", constantSpeedAuto);

    // logger.cmdCommandXEntry.append(translation.getX());
    // logger.cmdCommandYEntry.append(translation.getY());
    // logger.cmdCommandRotationEntry.append(rotation);
    
    // double translateX = translation.getX();
    // double translateY = translation.getY();
    //
    // double translateX_sgn = Math.signum(translateX);
    // double translateY_sgn = Math.signum(translateY);
    // double desaturatedX = Math.min(Math.abs(translateX), 1);
    // double desaturatedY = Math.min(Math.abs(translateY), 1);
    // translation = new Translation2d(translateX_sgn * desaturatedX, translateY_sgn * desaturatedY);

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
    // |Tx| < .5 degree
    // |Gyro error| < 1 degreee
    // return false;
    if (!isAuto)
      return false;
    double elapsed = Timer.getFPGATimestamp() - startTime;
    if (elapsed >= 20) {
      SmartDashboard.putBoolean("ended by time", true);
      return true;
    }

    SmartDashboard.putBoolean("ended by time", false);
    //FYI rotationerror will never be >1000, just hacky so it doesn't care about rotation error
    return Math.abs(txValue) < 1 && Math.abs(rotationError) < 1000 && Math.abs(distanceError) < 1 && elapsed >= 0.1;
  }
}
