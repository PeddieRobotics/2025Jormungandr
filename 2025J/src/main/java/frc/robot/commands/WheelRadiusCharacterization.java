// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.DriveConstants;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class WheelRadiusCharacterization extends Command {
 
  private Drivetrain drivetrain;
  private double currentGyro, initialGyro, deltaGyro;
  private double[] initialPositions, currentPositions;
  private double averageDeltaPositions;
  private double effectiveWheelRadius;
  private double drivebaseRadius;
  
  private double initialTime;
  private double currentTime;

  private boolean started;
  /**
   * Creates a new wheelRadiusCharacterization
   .
   *
   * @param subsystem The subsystem used by this command.
   */
  public WheelRadiusCharacterization() {
    drivetrain = Drivetrain.getInstance();
    currentGyro = 0;
    initialGyro = 0;
    deltaGyro = 0;
    initialPositions = new double[4];
    currentPositions = new double[4];
    averageDeltaPositions = 0;
    effectiveWheelRadius = 0;
    drivebaseRadius = Math.sqrt(Math.pow((DriveConstants.kTrackWidth/2),2) + Math.pow(DriveConstants.kWheelBase/2, 2)) ;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    started = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTime = Timer.getFPGATimestamp();
    started = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    drivetrain.drive(new Translation2d(0,0), 3, false, null);

    SmartDashboard.putBoolean("WHEEL CHARAC: started Calib", started);

    if (currentTime - initialTime > 3 && !started){
      initialGyro = drivetrain.getYaw() * Math.PI / 180;
      initialPositions = Arrays.copyOf(drivetrain.getSwerveModulePositionsRadians(), 4);
      started = true;
    }

    if (currentTime-initialTime > 5){
      averageDeltaPositions = 0;
      currentGyro = drivetrain.getYaw() * Math.PI / 180;
      deltaGyro = currentGyro - initialGyro;
      SmartDashboard.putNumber("WHEEL CHARAC: deltaGyro", deltaGyro);

      currentPositions = drivetrain.getSwerveModulePositionsRadians();
      SmartDashboard.putNumber("WHEEL CHARAC: current module 0", currentPositions[0]);
      SmartDashboard.putNumber("WHEEL CHARAC: initial module 0", initialPositions[0]);
      SmartDashboard.putNumber("WHEEL CHARAC: delta module 0", currentPositions[0]-initialPositions[0]);
      SmartDashboard.putNumber("WHEEL CHARAC: delta module 1", currentPositions[1]-initialPositions[1]);
      SmartDashboard.putNumber("WHEEL CHARAC: delta module 2", currentPositions[2]-initialPositions[2]);
      SmartDashboard.putNumber("WHEEL CHARAC: delta module 3", currentPositions[3]-initialPositions[3]);

      for(int i = 0; i < 4; i++){
        averageDeltaPositions += Math.abs((currentPositions[i] - initialPositions[i]));
      }
      averageDeltaPositions /= 4;

      effectiveWheelRadius = deltaGyro * drivebaseRadius / averageDeltaPositions;
      effectiveWheelRadius = Units.metersToInches(effectiveWheelRadius);

      // logger.deltaGyro.append(deltaGyro);
      // logger.averageDeltaPositions.append(averageDeltaPositions);
      // logger.effectiveWheelRadius.append(effectiveWheelRadius);
      SmartDashboard.putNumber("WHEEL CHARAC: averageDeltaPositions", averageDeltaPositions);
      SmartDashboard.putNumber("WHEEL CHARAC: drivebaseRadius", drivebaseRadius);
      SmartDashboard.putNumber("WHEEL CHARAC: effectiveWheelRadius",effectiveWheelRadius);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("WHEEL CHARAC: effectiveWheelRadius",effectiveWheelRadius);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}