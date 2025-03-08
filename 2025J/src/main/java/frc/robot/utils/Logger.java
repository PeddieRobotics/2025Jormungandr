package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Claw;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HPIntake;
// import frc.robot.subsystems.LimelightClimber;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFrontLeft;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.subsystems.LimelightFrontRight;
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
    // private Climber climber;

    Limelight[] limelights;

    private DataLog log = DataLogManager.getLog();
    private StringLogEntry commandEntry, superstructureCurrentStateEntry, superstructureRequestedStateEntry;
    private BooleanLogEntry LLIntakeHasTargetEntry, clawTopSensorEntry, clawBottomSensorEntry, clawAlgaeSensorEntry;
    private DoubleLogEntry gyroAngleEntry, gyroAngleEntryBlue, driveTrainSpeedEntry, driveTrainAngleEntry, driveTrainXEntry,
            driveTrainYEntry, driveTrainXVelEntry, driveTrainZAccEntry,
            driveTrainYVelEntry, driveTrainXAccEntry, driveTrainYAccEntry, driveTrainAngleVelEntry,
            armAngleEntry, armPositionEntry, armPositionSetpointEntry, armCANcoderPositionEntry,
            armSupplyCurrentEntry, armStatorCurrentEntry, armTorqueCurrentEntry, armVelocityEntry,
            clawPositionEntry, clawVelocityEntry, clawSupplyCurrentEntry, clawStatorCurrentEntry,
            elevatorPositionEntry, elevatorVelocityEntry, elevatorPositionSetpointEntry,
            elevatorMainMotorSupplyCurrentEntry, elevatorMainMotorStatorCurrentEntry, elevatorMainMotorTorqueCurrentEntry,
            elevatorFollowerMotorSupplyCurrentEntry, elevatorFollowerMotorStatorCurrentEntry, elevatorFollowerMotorTorqueCurrentEntry,
            leftClimberSupplyCurrentEntry, leftClimberStatorCurrentEntry, leftClimberTemperatureEntry,
            leftClimberPositionEntry,
            rightClimberSupplyCurrentEntry, rightClimberStatorCurrentEntry, rightClimberTemperatureEntry,
            rightClimberPosition;
    
    private DoubleLogEntry reefAlignXErrorEntry, reefAlignYErrorEntry, reefAlignRotationErrorEntry;
    private DoubleLogEntry reefAlignXEntry, reefAlignYEntry, reefAlignRotationEntry;
    private DoubleLogEntry HPAlignXErrorEntry, HPAlignYErrorEntry, HPAlignRotationErrorEntry;
    private DoubleLogEntry HPAlignXEntry, HPAlignYEntry, HPAlignRotationEntry;
    
    private DoubleArrayLogEntry moduleSpeedsEntry, modulePositionsEntry;
    // private StructPublisher<Pose2d> fusedOdometryEntry;

    private DoubleArrayLogEntry fusedOdometryEntry;
    private DoubleArrayLogEntry[] limelightMT2Entry;
    
    private DoubleLogEntry[] limelightTyDistanceEntry, limelightPoseDistanceEntry, limelightFilteredPoseDistanceEntry,
        limelightFilteredTyDistanceEntry, limelightNumOfApriltagEntry, limelightTxEntry, limelightTyEntry, limelightTargetEntry;
    // private List<StructPublisher<Pose2d>> limelightMT2Entry;

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
        superstructure = Superstructure.getInstance();
        // climber = Climber.getInstance();

        // Superstructure Logs
        superstructureCurrentStateEntry = new StringLogEntry(log, "/Superstructure/Current Superstructure State");
        superstructureRequestedStateEntry = new StringLogEntry(log, "/Superstructure/Requested Superstructure State");

        // Field Logs
        fusedOdometryEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Fused Odometry");

        // Drivetrain Logs
        gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");
        gyroAngleEntryBlue = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle Blue");

        driveTrainXAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain X Accel");
        driveTrainYAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Y Accel");
        driveTrainZAccEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Z Accel");

        driveTrainSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Speed");
        moduleSpeedsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Speeds");
        modulePositionsEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Positions");

        // Claw logs
        clawPositionEntry = new DoubleLogEntry(log, "/Claw/Claw Position");
        clawVelocityEntry = new DoubleLogEntry(log, "/Claw/Claw Velocity");
        clawSupplyCurrentEntry = new DoubleLogEntry(log, "/Claw/Claw Motor Supply Current");
        clawStatorCurrentEntry = new DoubleLogEntry(log, "/Claw/Claw Motor Stator Current");
        clawTopSensorEntry = new BooleanLogEntry(log, "/Claw/Top Sensor");
        clawBottomSensorEntry = new BooleanLogEntry(log, "/Claw/Bottom Sensor");
        clawAlgaeSensorEntry = new BooleanLogEntry(log, "Claw/Algae Sensor");

        // Elevator Logs
        elevatorPositionEntry = new DoubleLogEntry(log, "/Elevator/Elevator CANcoder Position");
        elevatorVelocityEntry = new DoubleLogEntry(log, "/Elevator/Elevator CANcoder Velocity");
        elevatorPositionSetpointEntry = new DoubleLogEntry(log, "/Elevator/Elevator Position Setpoint");
        elevatorMainMotorSupplyCurrentEntry = new DoubleLogEntry(log, "/Elevator/Elevator Main Motor Supply Current");
        elevatorMainMotorStatorCurrentEntry = new DoubleLogEntry(log, "/Elevator/Elevator Main Motor Stator Current");
        elevatorMainMotorTorqueCurrentEntry = new DoubleLogEntry(log, "/Elevator/Elevator Main Motor Torque Current");
        elevatorFollowerMotorSupplyCurrentEntry = new DoubleLogEntry(log, "/Elevator/Elevator Follower Motor Supply Current");
        elevatorFollowerMotorStatorCurrentEntry = new DoubleLogEntry(log, "/Elevator/Elevator Followre Motor Stator Current");
        elevatorFollowerMotorTorqueCurrentEntry = new DoubleLogEntry(log, "/Elevator/Elevator Follower Motor Torque Current");

        // Arm logs
        armAngleEntry = new DoubleLogEntry(log, "/Arm/Arm Angle");
        armSupplyCurrentEntry = new DoubleLogEntry(log, "/Arm/Arm Motor Supply Current");
        armStatorCurrentEntry = new DoubleLogEntry(log, "/Arm/Arm Motor Stator Current");
        armTorqueCurrentEntry = new DoubleLogEntry(log, "/Arm/Arm Motor Torque Current");
        armPositionEntry = new DoubleLogEntry(log, "/Arm/Arm Position");
        armVelocityEntry = new DoubleLogEntry(log, "/Arm/Arm Velocity");
        armCANcoderPositionEntry = new DoubleLogEntry(log, "/Arm/Arm CANcoder Position");
        armPositionSetpointEntry = new DoubleLogEntry(log, "/Arm/Arm Position Setpoint");

        // Climber logs
        leftClimberSupplyCurrentEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Supply Current");
        leftClimberStatorCurrentEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Stator Current");
        leftClimberTemperatureEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Temperature");
        leftClimberPositionEntry = new DoubleLogEntry(log, "/Climber/Left Climber Motor Position");

        rightClimberSupplyCurrentEntry = new DoubleLogEntry(log, "/Climber/Right Climber Motor Supply Current");
        rightClimberStatorCurrentEntry = new DoubleLogEntry(log, "/Climber/Right Climber Motor Stator Current");
        rightClimberTemperatureEntry = new DoubleLogEntry(log, "/Climber/Right Climber Motor Temperature");
        rightClimberPosition = new DoubleLogEntry(log, "/Climber/Right Climber Motor Position");
        
        // Limelight Logs
        limelights = new Limelight[] { 
            LimelightFrontLeft.getInstance(),
            LimelightFrontMiddle.getInstance(),
            LimelightFrontRight.getInstance(),
            LimelightBack.getInstance()
        };
        limelightMT2Entry = new DoubleArrayLogEntry[limelights.length];
        limelightTyDistanceEntry = new DoubleLogEntry[limelights.length];
        limelightPoseDistanceEntry = new DoubleLogEntry[limelights.length];
        limelightFilteredTyDistanceEntry = new DoubleLogEntry[limelights.length];
        limelightFilteredPoseDistanceEntry = new DoubleLogEntry[limelights.length];
        limelightNumOfApriltagEntry = new DoubleLogEntry[limelights.length];
        limelightTxEntry = new DoubleLogEntry[limelights.length];
        limelightTyEntry = new DoubleLogEntry[limelights.length];
        limelightTargetEntry = new DoubleLogEntry[limelights.length];

        for (int i = 0; i < limelights.length; i++) {
            String cameraName = limelights[i].getName();
            limelightTyDistanceEntry[i] = new DoubleLogEntry(log, "/Limelight/" + cameraName + " Ty Distance");
            limelightPoseDistanceEntry[i] = new DoubleLogEntry(log, "/Limelight/" + cameraName + " Pose Distance");
            limelightFilteredTyDistanceEntry[i] = new DoubleLogEntry(log, "/Limelight/" + cameraName + " Filtered Ty Distance");
            limelightFilteredPoseDistanceEntry[i] = new DoubleLogEntry(log, "/Limelight/" + cameraName + " Filtered Pose Distance");
            limelightNumOfApriltagEntry[i] = new DoubleLogEntry(log, "/Limelight/" + cameraName + " Number of AprilTags");
            limelightTxEntry[i] = new DoubleLogEntry(log, "/Limelight/" + cameraName + " Tx");
            limelightTyEntry[i] = new DoubleLogEntry(log, "/Limelight/" + cameraName + " Ty");
            // limelightMT2Entry.add(new StructPublisher<Pose2d>())
            limelightMT2Entry[i] = new DoubleArrayLogEntry(log, "/Limelight/" + cameraName + " MT2 Pose");
            limelightTargetEntry[i] = new DoubleLogEntry(log, "/Limelight/" + cameraName + " Best Target ID");
        }
        
        // Alignment Command Logs
        reefAlignXErrorEntry = new DoubleLogEntry(log, "/Alignment/Reef X Error");
        reefAlignYErrorEntry = new DoubleLogEntry(log, "/Alignment/Reef Y Error");
        reefAlignRotationErrorEntry = new DoubleLogEntry(log, "/Alignment/Reef Rotation Error");

        reefAlignXEntry = new DoubleLogEntry(log, "/Alignment/Reef Commanded X");
        reefAlignYEntry = new DoubleLogEntry(log, "/Alignment/Reef Commanded Y");
        reefAlignRotationEntry = new DoubleLogEntry(log, "/Alignment/Reef Commanded Rotation");

        HPAlignXErrorEntry = new DoubleLogEntry(log, "/Alignment/HP X Error");
        HPAlignYErrorEntry = new DoubleLogEntry(log, "/Alignment/HP Y Error");
        HPAlignRotationErrorEntry = new DoubleLogEntry(log, "/Alignment/HP Rotation Error");

        HPAlignXEntry = new DoubleLogEntry(log, "/Alignment/HP Commanded X");
        HPAlignYEntry = new DoubleLogEntry(log, "/Alignment/HP Commanded Y");
        HPAlignRotationEntry = new DoubleLogEntry(log, "/Alignment/HP Commanded Rotation");

        // Commands run
        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");
    }

    public void logEvent(String event, boolean isStart) {
        commandEntry.append(event + (isStart ? " Started" : " Ended"));
    }

    public void updateLogs() {

        // Drivetrain Logs
        updateDrivetrainLogs();

        // Superstructure Logs
        superstructureCurrentStateEntry.append(superstructure.getCurrentState().toString());
        superstructureRequestedStateEntry.append(superstructure.getRequestedState().toString());

        // Claw logs
        clawPositionEntry.append(claw.getPosition());
        // clawAccEntry.append(claw.getAcc());
        clawVelocityEntry.append(claw.getVelocity());
        clawSupplyCurrentEntry.append(claw.getMotorSupplyCurrent());
        clawStatorCurrentEntry.append(claw.getMotorStatorCurrent());
        clawTopSensorEntry.append(claw.getTopSensor());
        clawBottomSensorEntry.append(claw.getBottomSensor());
        clawAlgaeSensorEntry.append(claw.getAlgaeSensor());

        // Elevator Logs
        elevatorPositionEntry.append(elevator.getElevatorCANcoderPosition());
        elevatorVelocityEntry.append(elevator.getElevatorCANcoderVelocity());
        elevatorPositionSetpointEntry.append(elevator.getElevatorSetpoint());
        elevatorMainMotorSupplyCurrentEntry.append(elevator.getMainMotorSupplyCurrent());
        elevatorMainMotorStatorCurrentEntry.append(elevator.getMainMotorStatorCurrent());
        elevatorFollowerMotorSupplyCurrentEntry.append(elevator.getFollowerMotorSupplyCurrent());
        elevatorFollowerMotorStatorCurrentEntry.append(elevator.getFollowerMotorStatorCurrent());

        // Arm Logs
        armAngleEntry.append(arm.getArmAngleDegrees());
        armSupplyCurrentEntry.append(arm.getMotorSupplyCurrent());
        armStatorCurrentEntry.append(arm.getMotorStatorCurrent());
        armTorqueCurrentEntry.append(arm.getMotorTorqueCurrent());
        armPositionEntry.append(arm.getArmMotorEncoderPosition());
        armVelocityEntry.append(arm.getArmVelocity());
        armPositionSetpointEntry.append(arm.getArmSetpoint());
        armCANcoderPositionEntry.append(arm.getAbsoluteCANcoderPosition());

        // Climber Logs
        // leftClimberSupplyCurrentEntry.append(climber.getLeftClimberSupplyCurrent());
        // leftClimberStatorCurrentEntry.append(climber.getLeftClimberStatorCurrent());
        // leftClimberTemperatureEntry.append(climber.getLeftClimberTemperature());
        // leftClimberPositionEntry.append(climber.getLeftClimberPosition());

        // rightClimberSupplyCurrentEntry.append(climber.getRightClimberSupplyCurrent());
        // rightClimberStatorCurrentEntry.append(climber.getRightClimberStatorCurrent());
        // rightClimberTemperatureEntry.append(climber.getRightClimberTemperature());
        // rightClimberPosition.append(climber.getRightClimberPosition());

        // Limelight Logs
        for (int i = 0; i < limelights.length; i++) {
            limelightTyDistanceEntry[i].append(limelights[i].getDistanceTy());
            limelightPoseDistanceEntry[i].append(limelights[i].getDistanceEstimatedPose());
            limelightFilteredTyDistanceEntry[i].append(limelights[i].getFilteredDistanceTy());
            limelightFilteredPoseDistanceEntry[i].append(limelights[i].getFilteredDistanceEstimatedPose());
            limelightNumOfApriltagEntry[i].append(limelights[i].getNumberOfTagsSeen());
            limelightTxEntry[i].append(limelights[i].getTxAverage());
            limelightTyEntry[i].append(limelights[i].getTyAverage());
            limelightMT2Entry[i].append(pose2dToDoubleArray(limelights[i].getEstimatedPoseMT2()));
            limelightTargetEntry[i].append(limelights[i].getTargetID());
        }
    }

    private double[] pose2dToDoubleArray(Pose2d pose) {
        return new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        };
    }

    private double[] pose2dToDoubleArray(Optional<Pose2d> pose) {
        if (pose.isEmpty())
            return new double[] { 0, 0, 0 };
        return new double[] {
            pose.get().getX(),
            pose.get().getY(),
            pose.get().getRotation().getRadians()
        };
    }

    public void updateDrivetrainLogs() {
        gyroAngleEntry.append(DriverStation.isAutonomous() ? drivetrain.getHeadingForceAdjust() : drivetrain.getHeading());
        gyroAngleEntryBlue.append(DriverStation.isAutonomous() ? drivetrain.getHeadingBlueForceAdjust() : drivetrain.getHeadingBlue());

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

        fusedOdometryEntry.append(pose2dToDoubleArray(drivetrain.getPose()));
    }
    
    public void logAlignToReef(double xError, double yError, double rotationError, double x, double y, double rot) {
        reefAlignXErrorEntry.append(xError);
        reefAlignYErrorEntry.append(yError);
        reefAlignRotationErrorEntry.append(rotationError);

        reefAlignXEntry.append(x);
        reefAlignYEntry.append(y);
        reefAlignRotationEntry.append(rot);
    }
    public void logAlignToHP(double xError, double yError, double rotationError, double x, double y, double rot) {
        HPAlignXErrorEntry.append(xError);
        HPAlignYErrorEntry.append(yError);
        HPAlignRotationErrorEntry.append(rotationError);
        
        HPAlignXEntry.append(x);
        HPAlignYEntry.append(y);
        HPAlignRotationEntry.append(rot);
    }

    public void logScoreEvent(int level, double elevator, double arm) {
        logEvent("Score L" + level + " with elevator " + elevator + ", arm " + arm, false);
    }
}
