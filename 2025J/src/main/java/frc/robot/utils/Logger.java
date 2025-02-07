package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveModule;

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
    private StringLogEntry commandEntry;
    private DoubleLogEntry gyroAngleEntry, driveTrainSpeedEntry, driveTrainAngleEntry, driveTrainXEntry, driveTrainYEntry, driveTrainXVelEntry,driveTrainZAccEntry,
            driveTrainYVelEntry, driveTrainXAccEntry, driveTrainYAccEntry, driveTrainAngleVelEntry, armAngleEntry, armSpeedEntry,
            armAccEntry, armPositionEntry, armVelocityEntry, clawAngleEntry, clawAccEntry, clawPositionEntry, clawVelocityEntry, elevatorPositionEntry,
            elevatorVelocityEntry, elevatorAccEntry, elevatorSpeedEntry, hpIntakePositionEntry, hpIntakeVelocityEntry, hpIntakeAccEntry,
            superstructurePositionEntry, superstructureVelocityEntry, superstructureAccEntry, leftClimberSupplyCurrentEntry,
            leftClimberStatorCurrentEntry, leftClimberTemperatureEntry, leftClimberPositionEntry, rightClimberSupplyCurrentEntry,
            rightClimberStatorCurrentEntry, rightClimberTemperatureEntry, rightClimberPosition, fieldPositionEntry, botposeFieldPositionEntry, moduleSpeedsEntry, modulePositionsEntry;

    public static Logger getInstance() {
        if (instance == null) {
            instance = new Logger();
        }
        return instance;
    }

    public Logger() {
        drivetrain = Drivetrain.getInstance();
        // arm = Arm.getInstance();
        autonomous = Autonomous.getInstance();
        // claw = Claw.getInstance();
        // elevator = Elevator.getInstance();
        // hpIntake = HPIntake.getInstance();
        // superstructure = Superstructure.getInstance();
        // climber = Climber.getInstance();

        /*
         * Superstructure Logs
         */

        
        /*
         * Drivetrain Logs
         */

        gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");

        driveTrainXAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain X Accel");
        driveTrainYAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Y Accel");
        driveTrainZAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Z Accel");

        driveTrainSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Speed");
        fieldPositionEntry = new DoubleLogEntry(log, "/Field/Position");
        botposeFieldPositionEntry = new DoubleLogEntry(log, "/Field/Botpose position");
        moduleSpeedsEntry = new DoubleLogEntry(log, "/Drivetrain/Swerve Module Speeds");
        modulePositionsEntry = new DoubleLogEntry(log, "/Drivetrain/Swerve Module Positions");

   

        
        /*
         * Intake Logs
         */

        /*
         * Claw logs
         */


        /*
         * Arm logs
         */


        /*
         * Climber logs
         */


        /*
         * Limelight Logs
         */

        
        //Commands run
        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");
    }

    public void logEvent(String event, Boolean isStart){
        commandEntry.append(event + (isStart? "Started" : "Ended"));
    }

    public void updateLogs(){
        //updateDrivetrainLogs();
    }

    public void updateDrivetrainLogs(){
        gyroAngleEntry.append(drivetrain.getHeading());

        driveTrainXAccEntry.append(drivetrain.getGyroAccX());
        driveTrainYAccEntry.append(drivetrain.getGyroAccY());
        driveTrainZAccEntry.append(drivetrain.getGyroAccZ());

    }
}
