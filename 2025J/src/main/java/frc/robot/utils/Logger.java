package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
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
import frc.robot.subsystems.PVBack;
import frc.robot.subsystems.PVFrontLeft;
import frc.robot.subsystems.PVFrontMiddle;
import frc.robot.subsystems.PVFrontRight;
import frc.robot.subsystems.PVLeft;
import frc.robot.subsystems.PhotonVision;
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
    private PVBack pvBack;
    private PVFrontLeft pvFrontLeft;
    private PVFrontMiddle pvFrontMiddle;
    private PVFrontRight pvFrontRight;
    private PVLeft pvLeft;

    PhotonVision[] cameras;

    private DataLog log = DataLogManager.getLog();
    private StringLogEntry commandEntry, superstructureCurrentStateEntry, superstructureRequestedStateEntry;
    private BooleanLogEntry LLIntakeHasTargetEntry;
    private DoubleLogEntry gyroAngleEntry, driveTrainSpeedEntry, driveTrainAngleEntry, driveTrainXEntry,
            driveTrainYEntry, driveTrainXVelEntry, driveTrainZAccEntry,
            driveTrainYVelEntry, driveTrainXAccEntry, driveTrainYAccEntry, driveTrainAngleVelEntry,
            armAngleEntry, armSpeedEntry, armSupplyCurrentEntry, armStatorCurrentEntry, armTorqueCurrentEntry,
            armAccEntry, armPositionEntry, armVelocityEntry,
            clawPositionEntry, clawAccEntry, clawVelocityEntry, clawSupplyCurrentEntry, clawStatorCurrentEntry,
            elevatorPositionEntry,
            elevatorVelocityEntry, elevatorAccEntry, elevatorSpeedEntry,
            hpIntakePositionEntry, hpIntakeVelocityEntry, hpIntakeAccEntry, hpIntakeSupplyCurrentEntry,
            hpIntakeStatorCurrentEntry,
            leftClimberSupplyCurrentEntry, leftClimberStatorCurrentEntry, leftClimberTemperatureEntry,
            leftClimberPositionEntry,
            rightClimberSupplyCurrentEntry, rightClimberStatorCurrentEntry, rightClimberTemperatureEntry,
            rightClimberPosition;

    private DoubleArrayLogEntry fieldPositionEntry, botposeFieldPositionEntry, moduleSpeedsEntry, modulePositionsEntry;
    
    private DoubleLogEntry[] PV_TyDistanceEntry, PV_PoseDistanceEntry, PV_FilteredPoseDistanceEntry, PV_FilteredTyDistanceEntry,  
        PV_NumOfApriltagEntry, PV_TxEntry, PV_TyEntry;

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
        superstructure = Superstructure.getInstance();
        // climber = Climber.getInstance();
        //
        pvBack = PVBack.getInstance();
        pvFrontLeft = PVFrontLeft.getInstance();
        pvFrontMiddle = PVFrontMiddle.getInstance();
        pvFrontRight = PVFrontRight.getInstance();
        pvLeft = PVLeft.getInstance();

        cameras = new PhotonVision[] {
            pvBack, pvFrontLeft, pvFrontMiddle, pvFrontRight, pvLeft
        };

        /*
         * Superstructure Logs
         */

        superstructureCurrentStateEntry = new StringLogEntry(log, "/Superstructure/Current Superstructure State");
        superstructureRequestedStateEntry = new StringLogEntry(log, "/Superstructure/Requested Superstructure State");

        /*
         * Field Logs
         */

        fieldPositionEntry = new DoubleArrayLogEntry(log, "/Field/Position");
        botposeFieldPositionEntry = new DoubleArrayLogEntry(log, "/Field/Botpose position");

        /*
         * Drivetrain Logs
         */

        gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");

        driveTrainXAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain X Accel");
        driveTrainYAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Y Accel");
        driveTrainZAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Z Accel");

        driveTrainSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Speed");
        moduleSpeedsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Speeds");
        modulePositionsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Positions");

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

        clawPositionEntry = new DoubleLogEntry(log, "/Claw/Claw Position");
        clawAccEntry = new DoubleLogEntry(log, "/Claw/Claw Acceleration");
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
         * Photonvision Logs
         */
        
        PV_TyDistanceEntry = new DoubleLogEntry[5];
        PV_PoseDistanceEntry = new DoubleLogEntry[5];
        PV_FilteredTyDistanceEntry = new DoubleLogEntry[5];
        PV_FilteredPoseDistanceEntry = new DoubleLogEntry[5];
        PV_NumOfApriltagEntry = new DoubleLogEntry[5];
        PV_TxEntry = new DoubleLogEntry[5];
        PV_TyEntry = new DoubleLogEntry[5];

        for (int i = 0; i < 5; i++) {
            String cameraName = cameras[i].getName();
            PV_TyDistanceEntry[i] = new DoubleLogEntry(log, "/Camera/" + cameraName + " Ty Distance");
            PV_PoseDistanceEntry[i] = new DoubleLogEntry(log, "/Camera/" + cameraName + " Pose Distance");
            PV_FilteredTyDistanceEntry[i] = new DoubleLogEntry(log, "/Camera/" + cameraName + " Filtered Ty Distance");
            PV_FilteredPoseDistanceEntry[i] = new DoubleLogEntry(log, "/Camera/" + cameraName + " Filtered Pose Distance");
            PV_NumOfApriltagEntry[i] = new DoubleLogEntry(log, "/Camera/" + cameraName + " Number of AprilTags");
            PV_TxEntry[i] = new DoubleLogEntry(log, "/Camera/" + cameraName + " Tx");
            PV_TyEntry[i] = new DoubleLogEntry(log, "/Camera/" + cameraName + " Ty");
        }
    }

    public void logEvent(String event, Boolean isStart) {
        commandEntry.append(event + (isStart ? "Started" : "Ended"));
    }

    public void updateLogs() {

        /*
         * Drivetrain Logs
         */
        updateDrivetrainLogs();

        /*
         * Superstructure Logs
         */

        superstructureCurrentStateEntry.append(superstructure.getCurrentState().toString());
        superstructureRequestedStateEntry.append(superstructure.getRequestedState().toString());

        /*
         * Intake Logs
         */

        // hpIntakePositionEntry.append(hpIntake.getIntakePosition());
        // hpIntakeVelocityEntry.append(hpIntake.getIntakeVelocity());
        // // hpIntakeAccEntry.append(hpIntake.getIntakeAcc());
        // hpIntakeSupplyCurrentEntry.append(hpIntake.getMotorSupplyCurrent());
        // hpIntakeStatorCurrentEntry.append(hpIntake.getMotorStatorCurrent());
        /*
         * Claw logs
         */

        // clawPositionEntry.append(claw.getPosition());
        // // clawAccEntry.append(claw.getAcc());
        // clawVelocityEntry.append(claw.getVelocity());
        // clawSupplyCurrentEntry.append(claw.getMotorSupplyCurrent());
        // clawStatorCurrentEntry.append(claw.getMotorStatorCurrent());

        /*
         * Elevator Logs
         */

        // elevatorPositionEntry.append(elevator.getPosition());
        // elevatorVelocityEntry.append(elevator.getVelocity());
        // elevatorAccEntry.append(elevator.getAcc());

        /*
         * Arm logs
         */

        // armAngleEntry.append(arm.getArmAngleDegrees());
        // // armAccEntry.append(arm.getAcc());
        // armSupplyCurrentEntry.append(arm.getMotorSupplyCurrent());
        // armStatorCurrentEntry.append(arm.getMotorStatorCurrent());
        // armTorqueCurrentEntry.append(arm.getMotorTorqueCurrent());
        // armPositionEntry.append(arm.getArmPosition());
        // armVelocityEntry.append(arm.getArmVelocity());

        /*
         * Climber logs
         */

        // leftClimberSupplyCurrentEntry.append(climber.getLeftClimberSupplyCurrent());
        // leftClimberStatorCurrentEntry.append(climber.getLeftClimberStatorCurrent());
        // leftClimberTemperatureEntry.append(climber.getLeftClimberTemperature());
        // leftClimberPositionEntry.append(climber.getLeftClimberPosition());

        // rightClimberSupplyCurrentEntry.append(climber.getRightClimberSupplyCurrent());
        // rightClimberStatorCurrentEntry.append(climber.getRightClimberStatorCurrent());
        // rightClimberTemperatureEntry.append(climber.getRightClimberTemperature());
        // rightClimberPosition.append(climber.getRightClimberPosition());

        // PhotonVision Logs
        for (int i = 0; i < 5; i++) {
            PV_TyDistanceEntry[i].append(cameras[i].getDistanceTy());
            PV_PoseDistanceEntry[i].append(cameras[i].getDistanceEstimatedPose());
            PV_FilteredTyDistanceEntry[i].append(cameras[i].getFilteredDistanceTy());
            PV_FilteredPoseDistanceEntry[i].append(cameras[i].getFilteredDistanceEstimatedPose());
            PV_NumOfApriltagEntry[i].append(cameras[i].getNumberOfTagsSeen());
            PV_TxEntry[i].append(cameras[i].getTxAverage());
            PV_TyEntry[i].append(cameras[i].getTyAverage());
        }

        // Commands run
        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");
    }

    public void updateDrivetrainLogs() {
        gyroAngleEntry.append(drivetrain.getHeading());

        driveTrainXAccEntry.append(drivetrain.getGyroAccX());
        driveTrainYAccEntry.append(drivetrain.getGyroAccY());
        driveTrainZAccEntry.append(drivetrain.getGyroAccZ());
        driveTrainSpeedEntry.append(drivetrain.getSpeed());

        SwerveModuleState[] moduleStates = drivetrain.getSwerveModuleStates();
        double[] swerveModulePositions = { moduleStates[0].angle.getDegrees(), moduleStates[1].angle.getDegrees(),
                moduleStates[2].angle.getDegrees(), moduleStates[3].angle.getDegrees() };
        double[] swerveModuleSpeeds = { moduleStates[0].speedMetersPerSecond, moduleStates[1].speedMetersPerSecond,
                moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond };
        moduleSpeedsEntry.append(swerveModuleSpeeds);
        modulePositionsEntry.append(swerveModulePositions);

    }
}
