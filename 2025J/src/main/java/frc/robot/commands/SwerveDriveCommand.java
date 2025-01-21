// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.DriverOI;

public class SwerveDriveCommand extends Command {

  private Drivetrain drivetrain;
  private DriverOI oi;

  public SwerveDriveCommand() {
    drivetrain = Drivetrain.getInstance();

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    oi = DriverOI.getInstance();
  }

  @Override
  public void execute() {
    Translation2d translation = oi.getSwerveTranslation();
    double rotation = oi.getRotation();

    drivetrain.drive(translation, rotation, true, new Translation2d(0, 0));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
