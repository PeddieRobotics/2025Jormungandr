package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HPIntake;
import frc.robot.subsystems.Superstructure;

public class Logger {
    private static Logger instance;
    private Drivetrain drivetrain;
    private Arm arm;
    private Autonomous autonomous;
    private Claw claw;
    private Elevator elevator;
    private HPIntake hpIntake;
    private Superstructure superstructure;
    private Climber climber;
     
    private DataLog log = DataLogManager.getLog();
    private StringLogEntry commandEntry, superstructureCurrentStateEntry, superstructureRequestedStateEntry;
    private BooleanLogEntry LLIntakeHasTargetEntry;
    private DoubleLogEntry gyroAngleEntry, driveTrainSpeedEntry, driveTrainAngleEntry, driveTrainXEntry, driveTrainYEntry, driveTrainXVelEntry,driveTrainZAccEntry,
            driveTrainYVelEntry, driveTrainXAccEntry, driveTrainYAccEntry, driveTrainAngleVelEntry, 
            armAngleEntry, armSpeedEntry, armSupplyCurrentEntry, armStatorCurrentEntry, armTorqueCurrentEntry,armAccEntry, armPositionEntry, armVelocityEntry, 
            clawAngleEntry, clawAccEntry, clawPositionEntry, clawVelocityEntry, clawSupplyCurrentEntry, clawStatorCurrentEntry,elevatorPositionEntry,
            elevatorVelocityEntry, elevatorAccEntry, elevatorSpeedEntry, 
            hpIntakePositionEntry, hpIntakeVelocityEntry, hpIntakeAccEntry, hpIntakeSupplyCurrentEntry, hpIntakeStatorCurrentEntry,
            LLShooterDistanceEntry, LLShooterNumOfApriltagEntry, LLShooterTxEntry, LLIntakeTxEntry, 
            leftClimberSupplyCurrentEntry,leftClimberStatorCurrentEntry, leftClimberTemperatureEntry, leftClimberPositionEntry, 
            rightClimberSupplyCurrentEntry, rightClimberStatorCurrentEntry, rightClimberTemperatureEntry, rightClimberPosition, 
            fieldPositionEntry, botposeFieldPositionEntry, moduleSpeedsEntry, modulePositionsEntry;

    public static Logger getInstance() {
        if (instance == null) {
            instance = new Logger();
        }
        return instance;
    }

    public Logger() {
        drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        autonomous = Autonomous.getInstance();
        claw = Claw.getInstance();
        elevator = Elevator.getInstance();
        hpIntake = HPIntake.getInstance();
        superstructure = Superstructure.getInstance();
        climber = Climber.getInstance();

        /*
         * Superstructure Logs
         */

        superstructureCurrentStateEntry = new StringLogEntry(log, "/Superstructure/Current Superstructure State");
        superstructureRequestedStateEntry = new StringLogEntry(log, "/Superstructure/Requested Superstructure State");

        /*
         * Field Logs
         */

        fieldPositionEntry = new DoubleLogEntry(log, "/Field/Position");
        botposeFieldPositionEntry = new DoubleLogEntry(log, "/Field/Botpose position");
        
        /*
         * Drivetrain Logs
         */

        gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");

        driveTrainXAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain X Accel");
        driveTrainYAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Y Accel");
        driveTrainZAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Z Accel");

        driveTrainSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Speed");
        moduleSpeedsEntry = new DoubleLogEntry(log, "/Drivetrain/Swerve Module Speeds");
        modulePositionsEntry = new DoubleLogEntry(log, "/Drivetrain/Swerve Module Positions");

   

        
        /*
         * Intake Logs
         */

        hpIntakePositionEntry = new DoubleLogEntry(log, "/Intake/HP Intake Position");
        hpIntakeVelocityEntry = new DoubleLogEntry(log, "/Intake/HP Intake Velocity");
        hpIntakeAccEntry = new DoubleLogEntry(log, "/Intake/HP Intake Acceleration");
        hpIntakeSupplyCurrentEntry = new DoubleLogEntry(log, "/Intake/HP Intake Motor Supply Current");
        hpIntakeStatorCurrentEntry = new DoubleLogEntry(log, "/Intake/HP Intake Motor Stator Current");
        /*
         * Claw logs
         */

        clawAngleEntry = new DoubleLogEntry(log, "/Claw/Claw Angle");
        clawAccEntry = new DoubleLogEntry(log, "/Claw/Claw Acceleration");
        clawPositionEntry = new DoubleLogEntry(log, "/Claw/Claw Position");
        clawVelocityEntry = new DoubleLogEntry(log, "/Claw/Claw Velocity");
        clawSupplyCurrentEntry = new DoubleLogEntry(log, "/Claw/Claw Motor Supply Current");
        clawStatorCurrentEntry = new DoubleLogEntry(log, "/Claw/Claw Motor Stator Current");



        /*
         * Elevator Logs
         */

        elevatorPositionEntry = new DoubleLogEntry(log, "/Elevator/Elevator Position");
        elevatorVelocityEntry = new DoubleLogEntry(log, "/Elevator/Elevator Velocity");
        elevatorAccEntry = new DoubleLogEntry(log, "/Elevator/Elevator Acceleration");

        /*
         * Arm logs
         */ 

        armAngleEntry = new DoubleLogEntry(log, "/Arm/Arm Angle");
        armAccEntry = new DoubleLogEntry(log, "/Arm/Arm Acceleration");
        armSupplyCurrentEntry = new DoubleLogEntry(log, "/Arm/Arm Motor Supply Current");
        armStatorCurrentEntry = new DoubleLogEntry(log, "/Arm/Arm Motor Stator Current");
        armTorqueCurrentEntry = new DoubleLogEntry(log, "/Arm/Arm Motor Torque Current");
        armPositionEntry = new DoubleLogEntry(log, "/Arm/Arm Position");
        armVelocityEntry = new DoubleLogEntry(log, "/Arm/Arm Velocity");

        /*
         * Climber logs
         */

        leftClimberSupplyCurrentEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Supply Current");
        leftClimberStatorCurrentEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Stator Current");
        leftClimberTemperatureEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Temperature");
        leftClimberPositionEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Position");

        rightClimberSupplyCurrentEntry = new DoubleLogEntry(log, "/Climber/Right Climber Motor Supply Current");
        rightClimberStatorCurrentEntry = new DoubleLogEntry(log, "/Climber/Right Climber Motor Stator Current");
        rightClimberTemperatureEntry = new DoubleLogEntry(log, "/Climber/Right Climber Motor Temperature");
        rightClimberPosition = new DoubleLogEntry(log, "/Climber/Right Climber Motor Position");

        /*
         * Limelight Logs
         */

        LLShooterDistanceEntry = new DoubleLogEntry(log, "/Limelight Shooter/Distance");
        LLShooterNumOfApriltagEntry = new DoubleLogEntry(log, "/Limelight Shooter/Number of Apriltags");
        LLShooterTxEntry = new DoubleLogEntry(log, "/Limelight Shooter/Tx");
        
        LLIntakeHasTargetEntry = new BooleanLogEntry(log, "/Limelight Intake/Has Target");
        LLIntakeTxEntry = new DoubleLogEntry(log, "/Limelight Intake/Tx");

        
        //Commands run
        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");
    }

    public void logEvent(String event, Boolean isStart){
        commandEntry.append(event + (isStart? "Started" : "Ended"));
    }

    public void updateLogs(){

        /*
         * Superstructure Logs
         */

        superstructureCurrentStateEntry.append(superstructure.getCurrentState().toString());
        superstructureRequestedStateEntry.append(superstructure.getRequestedState().toString());
         
        /*
         * Drivetrain Logs
         */
        //updateDrivetrainLogs();
 
         driveTrainXAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain X Accel");
         driveTrainYAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Y Accel");
         driveTrainZAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Z Accel");
 
         driveTrainSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Speed");
         moduleSpeedsEntry = new DoubleLogEntry(log, "/Drivetrain/Swerve Module Speeds");
         modulePositionsEntry = new DoubleLogEntry(log, "/Drivetrain/Swerve Module Positions");
 
    
 
         
         /*
          * Intake Logs
          */
 
         hpIntakePositionEntry = new DoubleLogEntry(log, "/Intake/HP Intake Position");
         hpIntakeVelocityEntry = new DoubleLogEntry(log, "/Intake/HP Intake Velocity");
         hpIntakeAccEntry = new DoubleLogEntry(log, "/Intake/HP Intake Acceleration");
         hpIntakeSupplyCurrentEntry = new DoubleLogEntry(log, "/Intake/HP Intake Motor Supply Current");
         hpIntakeStatorCurrentEntry = new DoubleLogEntry(log, "/Intake/HP Intake Motor Stator Current");
         /*
          * Claw logs
          */
 
         clawAngleEntry = new DoubleLogEntry(log, "/Claw/Claw Angle");
         clawAccEntry = new DoubleLogEntry(log, "/Claw/Claw Acceleration");
         clawPositionEntry = new DoubleLogEntry(log, "/Claw/Claw Position");
         clawVelocityEntry = new DoubleLogEntry(log, "/Claw/Claw Velocity");
         clawSupplyCurrentEntry = new DoubleLogEntry(log, "/Claw/Claw Motor Supply Current");
         clawStatorCurrentEntry = new DoubleLogEntry(log, "/Claw/Claw Motor Stator Current");
 
 
 
         /*
          * Elevator Logs
          */
 
         elevatorPositionEntry = new DoubleLogEntry(log, "/Elevator/Elevator Position");
         elevatorVelocityEntry = new DoubleLogEntry(log, "/Elevator/Elevator Velocity");
         elevatorAccEntry = new DoubleLogEntry(log, "/Elevator/Elevator Acceleration");
 
         /*
          * Arm logs
          */ 
 
         armAngleEntry = new DoubleLogEntry(log, "/Arm/Arm Angle");
         armAccEntry = new DoubleLogEntry(log, "/Arm/Arm Acceleration");
         armSupplyCurrentEntry = new DoubleLogEntry(log, "/Arm/Arm Motor Supply Current");
         armStatorCurrentEntry = new DoubleLogEntry(log, "/Arm/Arm Motor Stator Current");
         armTorqueCurrentEntry = new DoubleLogEntry(log, "/Arm/Arm Motor Torque Current");
         armPositionEntry = new DoubleLogEntry(log, "/Arm/Arm Position");
         armVelocityEntry = new DoubleLogEntry(log, "/Arm/Arm Velocity");
 
         /*
          * Climber logs
          */
 
         leftClimberSupplyCurrentEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Supply Current");
         leftClimberStatorCurrentEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Stator Current");
         leftClimberTemperatureEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Temperature");
         leftClimberPositionEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Position");
 
         rightClimberSupplyCurrentEntry = new DoubleLogEntry(log, "/Climber/Right Climber Motor Supply Current");
         rightClimberStatorCurrentEntry = new DoubleLogEntry(log, "/Climber/Right Climber Motor Stator Current");
         rightClimberTemperatureEntry = new DoubleLogEntry(log, "/Climber/Right Climber Motor Temperature");
         rightClimberPosition = new DoubleLogEntry(log, "/Climber/Right Climber Motor Position");
 
         /*
          * Limelight Logs
          */
 
         LLShooterDistanceEntry = new DoubleLogEntry(log, "/Limelight Shooter/Distance");
         LLShooterNumOfApriltagEntry = new DoubleLogEntry(log, "/Limelight Shooter/Number of Apriltags");
         LLShooterTxEntry = new DoubleLogEntry(log, "/Limelight Shooter/Tx");
         
         LLIntakeHasTargetEntry = new BooleanLogEntry(log, "/Limelight Intake/Has Target");
         LLIntakeTxEntry = new DoubleLogEntry(log, "/Limelight Intake/Tx");
 
         
         //Commands run
         commandEntry = new StringLogEntry(log, "/Commands/Commands Run");
    }

    public void updateDrivetrainLogs(){
        gyroAngleEntry.append(drivetrain.getHeading());

        driveTrainXAccEntry.append(drivetrain.getGyroAccX());
        driveTrainYAccEntry.append(drivetrain.getGyroAccY());
        driveTrainZAccEntry.append(drivetrain.getGyroAccZ());

    }
}
