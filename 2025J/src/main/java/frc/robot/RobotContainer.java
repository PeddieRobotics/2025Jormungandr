// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HPIntake;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightClimber;
import frc.robot.subsystems.LimelightFrontLeft;
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.subsystems.Superstructure;
import frc.robot.utils.CalculateReefTarget;
import frc.robot.utils.DriverOI;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ScoreConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    private Arm arm;
    private Claw claw;
    private Climber climber;
    private Drivetrain drivetrain;
    private Superstructure superstructure;
    private DriverOI driverOI;
    private OperatorOI operatorOI;
    private Elevator elevator;
    private HPIntake hpIntake;
    private LimelightBack llBack;
    private LimelightFrontLeft llFrontLeft;
    private LimelightFrontRight llFrontRight;
    private LimelightClimber llClimber;
    private Autonomous autonomous;
    // private Lights lights;

    public RobotContainer() {
        arm = Arm.getInstance();
        claw = Claw.getInstance();
        climber = Climber.getInstance();
        drivetrain = Drivetrain.getInstance();
        elevator = Elevator.getInstance();
        hpIntake = HPIntake.getInstance();
        superstructure = Superstructure.getInstance();
        driverOI = DriverOI.getInstance();
        operatorOI = OperatorOI.getInstance();
        llBack = LimelightBack.getInstance();
        llFrontLeft = LimelightFrontLeft.getInstance();
        llFrontRight = LimelightFrontRight.getInstance();
        llClimber = LimelightClimber.getInstance();
        autonomous = Autonomous.getInstance();
        // lights = Lights.getInstance();





        CalculateReefTarget.initBlue();
        CalculateReefTarget.initRed();

        drivetrain.setDefaultCommand(new SwerveDriveCommand());

        SmartDashboard.putBoolean("Align: Smart Target Finding", true);
        SmartDashboard.putBoolean("Align: Auto Score", true);
        SmartDashboard.putBoolean("Align: Auto Prep", true);

        SmartDashboard.putBoolean("L1 Flag", false);
        SmartDashboard.putBoolean("L2 Flag", false);
        SmartDashboard.putBoolean("L3 Flag", false);
        SmartDashboard.putBoolean("L4 Flag", true);

        // SmartDashboard.putNumber("RemoveAlgae: elevator fast", -0.6);
        // SmartDashboard.putNumber("RemoveAlgae: elevator slow", -0.3);
        // SmartDashboard.putNumber("RemoveAlgae: slow threshold", 1.0);
        SmartDashboard.putBoolean("RemoveAlgae: high?", false);
        // SmartDashboard.putNumber("RemoveAlgae: thing", 0.3);

        SmartDashboard.putNumber("L1AlignLeft: lateral offset", 0.1);
        SmartDashboard.putNumber("L1AlignLeft: back offset", 0.55);
        SmartDashboard.putNumber("L1AlignLeft: angle offset", 20.0);

        SmartDashboard.putNumber("L1AlignRight: lateral offset", -0.1);
        SmartDashboard.putNumber("L1AlignRight: back offset", 0.55);
        SmartDashboard.putNumber("L1AlignRight: angle offset", -20.0);

        // SmartDashboard.putNumber("Tuning: L1Position", ScoreConstants.kElevatorL1ScorePosition);
        // SmartDashboard.putNumber("Tuning: L1Speed", ClawConstants.kCoralL1OuttakeSpeed);

        SmartDashboard.putNumber("Elevator: Global Offset", 0);
        SmartDashboard.putNumber("Drive: cardinal scale", DriveConstants.kCardinalDirectionSpeedScale);

        SmartDashboard.putNumber("Scoring Pose Offset", 0);

        SmartDashboard.putBoolean("Use Milstein Poles?", false);
    }
}
