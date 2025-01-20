// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HPIntake;
import frc.robot.utils.DriverOI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Arm arm;
  private Autonomous autonomous;
  private Claw claw;
  private Drivetrain drivetrain;
  private DriverOI driverOI;
  private Elevator elevator;
  private HPIntake hpIntake;

  public RobotContainer() {
    arm = Arm.getInstance();
    claw = Claw.getInstance();
    autonomous = Autonomous.getInstance();
    drivetrain = Drivetrain.getInstance();
    driverOI = DriverOI.getInstance();
    elevator = Elevator.getInstance();
    hpIntake = HPIntake.getInstance();

    SmartDashboard.putData("Auto Routines", autonomous.getAutoChooser());
    drivetrain.setDefaultCommand(new SwerveDriveCommand());
  }

  public Command getAutonomousCommand() {
    return Autonomous.getAutonomousCommand();
  }
}
