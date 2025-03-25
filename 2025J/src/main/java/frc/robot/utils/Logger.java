package frc.robot.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
import frc.robot.subsystems.LimelightFrontRight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveModule;

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
    private DoubleLogEntry gyroAngleEntry, gyroAngleEntryBlue, driveTrainSpeedDesiredEntry, driveTrainAngleEntry, driveTrainXEntry,
            driveTrainYEntry, driveTrainXVelEntry, driveTrainZAccEntry,
            driveTrainYVelEntry, driveTrainXAccEntry, driveTrainYAccEntry, driveTrainAngleVelEntry,
            armAngleEntry, armPositionEntry, armPositionSetpointEntry, armCANcoderPositionEntry,
            armSupplyCurrentEntry, armStatorCurrentEntry, armTorqueCurrentEntry, armVelocityEntry,
            clawCoralMotorPositionEntry, clawCoralMotorVelocityEntry, clawCoralMotorSupplyCurrentEntry, clawCoralMotorStatorCurrentEntry,
            elevatorPositionEntry, elevatorVelocityEntry, elevatorPositionSetpointEntry,
            elevatorMainMotorSupplyCurrentEntry, elevatorMainMotorStatorCurrentEntry, elevatorMainMotorTorqueCurrentEntry,
            elevatorFollowerMotorSupplyCurrentEntry, elevatorFollowerMotorStatorCurrentEntry, elevatorFollowerMotorTorqueCurrentEntry,
            leftClimberSupplyCurrentEntry, leftClimberStatorCurrentEntry, leftClimberTemperatureEntry,
            leftClimberPositionEntry,
            rightClimberSupplyCurrentEntry, rightClimberStatorCurrentEntry, rightClimberTemperatureEntry,
            rightClimberPosition;
    
    private DoubleLogEntry reefAlignLateralErrorEntry, reefAlignDepthErrorEntry, reefAlignRotationErrorEntry;
    private DoubleLogEntry reefAlignLateralEntry, reefAlignDepthEntry, reefAlignRotationEntry;
    private DoubleLogEntry HPAlignLateralErrorEntry, HPAlignDepthErrorEntry, HPAlignRotationErrorEntry;
    private DoubleLogEntry HPAlignLateralEntry, HPAlignDepthEntry, HPAlignRotationEntry;
    
    private DoubleArrayLogEntry moduleSpeedsDesiredEntry, modulePositionsDesiredEntry;
    private DoubleArrayLogEntry moduleSpeedsActualEntry, modulePositionsActualEntry;
    // private StructPublisher<Pose2d> fusedOdometryEntry;

    private DoubleArrayLogEntry fusedOdometryEntry;
    private DoubleArrayLogEntry[] limelightMT2Entry;
    
    private DoubleLogEntry[] limelightTyDistanceEntry, limelightPoseDistanceEntry, limelightFilteredPoseDistanceEntry,
        limelightFilteredTyDistanceEntry, limelightNumOfApriltagEntry, limelightTxEntry, limelightTyEntry, limelightTargetEntry;

    private Map<Integer, DoubleLogEntry> moduleDriveSupplyCurrentEntry, moduleSteerSupplyCurrentEntry;
    private Map<Integer, DoubleLogEntry> moduleDriveStatorCurrentEntry, moduleSteerStatorCurrentEntry;
    private Map<Integer, DoubleLogEntry> moduleCanCoderPositionEntry;

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

        driveTrainSpeedDesiredEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Desired Speed");
        moduleSpeedsDesiredEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Desired Speeds");
        modulePositionsDesiredEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Desired Positions");

        moduleSpeedsActualEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Actual Speeds");
        modulePositionsActualEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Actual Positions");

        // Claw logs
        clawCoralMotorPositionEntry = new DoubleLogEntry(log, "/Claw/Coral Motor Position");
        clawCoralMotorVelocityEntry = new DoubleLogEntry(log, "/Claw/Coral Motor Velocity");
        clawCoralMotorSupplyCurrentEntry = new DoubleLogEntry(log, "/Claw/Coral Motor Supply Current");
        clawCoralMotorStatorCurrentEntry = new DoubleLogEntry(log, "/Claw/Coral Motor Stator Current");
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
            // LimelightFrontMiddle.getInstance(),
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
        reefAlignLateralErrorEntry = new DoubleLogEntry(log, "/Alignment/Reef Lateral Error");
        reefAlignDepthErrorEntry = new DoubleLogEntry(log, "/Alignment/Reef Depth Error");
        reefAlignRotationErrorEntry = new DoubleLogEntry(log, "/Alignment/Reef Rotation Error");

        reefAlignLateralEntry = new DoubleLogEntry(log, "/Alignment/Reef Commanded Lateral");
        reefAlignDepthEntry = new DoubleLogEntry(log, "/Alignment/Reef Commanded Depth");
        reefAlignRotationEntry = new DoubleLogEntry(log, "/Alignment/Reef Commanded Rotation");

        HPAlignLateralErrorEntry = new DoubleLogEntry(log, "/Alignment/HP Lateral Error");
        HPAlignDepthErrorEntry = new DoubleLogEntry(log, "/Alignment/HP Depth Error");
        HPAlignRotationErrorEntry = new DoubleLogEntry(log, "/Alignment/HP Rotation Error");

        HPAlignLateralEntry = new DoubleLogEntry(log, "/Alignment/HP Commanded Lateral");
        HPAlignDepthEntry = new DoubleLogEntry(log, "/Alignment/HP Commanded Depth");
        HPAlignRotationEntry = new DoubleLogEntry(log, "/Alignment/HP Commanded Rotation");

        // Commands run
        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");

        // Swerve Module Logs
        moduleDriveSupplyCurrentEntry = new HashMap<>();
        moduleSteerSupplyCurrentEntry = new HashMap<>();

        moduleDriveSupplyCurrentEntry.put(12, new DoubleLogEntry(log, "/Modules/12 Drive Supply Current"));
        moduleDriveSupplyCurrentEntry.put(22, new DoubleLogEntry(log, "/Modules/22 Drive Supply Current"));
        moduleDriveSupplyCurrentEntry.put(32, new DoubleLogEntry(log, "/Modules/32 Drive Supply Current"));
        moduleDriveSupplyCurrentEntry.put(42, new DoubleLogEntry(log, "/Modules/42 Drive Supply Current"));

        moduleSteerSupplyCurrentEntry.put(12, new DoubleLogEntry(log, "/Modules/12 Steer Supply Current"));
        moduleSteerSupplyCurrentEntry.put(22, new DoubleLogEntry(log, "/Modules/22 Steer Supply Current"));
        moduleSteerSupplyCurrentEntry.put(32, new DoubleLogEntry(log, "/Modules/32 Steer Supply Current"));
        moduleSteerSupplyCurrentEntry.put(42, new DoubleLogEntry(log, "/Modules/42 Steer Supply Current"));


        
        moduleDriveStatorCurrentEntry = new HashMap<>();
        moduleSteerStatorCurrentEntry = new HashMap<>();

        moduleDriveStatorCurrentEntry.put(12, new DoubleLogEntry(log, "/Modules/12 Drive Stator Current"));
        moduleDriveStatorCurrentEntry.put(22, new DoubleLogEntry(log, "/Modules/22 Drive Stator Current"));
        moduleDriveStatorCurrentEntry.put(32, new DoubleLogEntry(log, "/Modules/32 Drive Stator Current"));
        moduleDriveStatorCurrentEntry.put(42, new DoubleLogEntry(log, "/Modules/42 Drive Stator Current"));

        moduleSteerStatorCurrentEntry.put(12, new DoubleLogEntry(log, "/Modules/12 Steer Stator Current"));
        moduleSteerStatorCurrentEntry.put(22, new DoubleLogEntry(log, "/Modules/22 Steer Stator Current"));
        moduleSteerStatorCurrentEntry.put(32, new DoubleLogEntry(log, "/Modules/32 Steer Stator Current"));
        moduleSteerStatorCurrentEntry.put(42, new DoubleLogEntry(log, "/Modules/42 Steer Stator Current"));



        moduleCanCoderPositionEntry = new HashMap<>();

        moduleCanCoderPositionEntry.put(12, new DoubleLogEntry(log, "Modules/12 CANCoder Position"));
        moduleCanCoderPositionEntry.put(22, new DoubleLogEntry(log, "Modules/22 CANCoder Position"));
        moduleCanCoderPositionEntry.put(32, new DoubleLogEntry(log, "Modules/32 CANCoder Position"));
        moduleCanCoderPositionEntry.put(42, new DoubleLogEntry(log, "Modules/42 CANCoder Position"));
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
        clawCoralMotorPositionEntry.append(claw.getCoralMotorPosition());
        clawCoralMotorVelocityEntry.append(claw.getCoralMotorVelocity());
        clawCoralMotorSupplyCurrentEntry.append(claw.getCoralMotorSupplyCurrent());
        clawCoralMotorStatorCurrentEntry.append(claw.getCoralMotorStatorCurrent());
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
        driveTrainSpeedDesiredEntry.append(drivetrain.getSpeed());

        SwerveModule[] modules = drivetrain.getSwerveModules();
        SwerveModuleState[] moduleStates = drivetrain.getSwerveModuleStates();
        double[] swerveModulePositions = { moduleStates[0].angle.getDegrees(), moduleStates[1].angle.getDegrees(),
                moduleStates[2].angle.getDegrees(), moduleStates[3].angle.getDegrees() };
        double[] swerveModuleSpeeds = { moduleStates[0].speedMetersPerSecond, moduleStates[1].speedMetersPerSecond,
                moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond };
                
        double[] swerveModulePositionsActual = { modules[0].getAngle(), modules[1].getAngle(),
                modules[2].getAngle(), modules[3].getAngle() };
        double[] swerveModuleSpeedsActual = { modules[0].getVelocity(), modules[1].getVelocity(),
                modules[2].getVelocity(), modules[3].getVelocity() };

        moduleSpeedsDesiredEntry.append(swerveModuleSpeeds);
        modulePositionsDesiredEntry.append(swerveModulePositions);
        moduleSpeedsActualEntry.append(swerveModuleSpeedsActual);
        modulePositionsActualEntry.append(swerveModulePositionsActual);

        fusedOdometryEntry.append(pose2dToDoubleArray(drivetrain.getPose()));
    }
    
    public void logAlignToReef(double lateralError, double depthError, double rotationError, double x, double y, double rot) {
        reefAlignLateralErrorEntry.append(lateralError);
        reefAlignDepthErrorEntry.append(depthError);
        reefAlignRotationErrorEntry.append(rotationError);

        reefAlignLateralEntry.append(x);
        reefAlignDepthEntry.append(y);
        reefAlignRotationEntry.append(rot);
    }
    public void logAlignToHP(double lateralError, double depthError, double rotationError, double x, double y, double rot) {
        HPAlignLateralErrorEntry.append(lateralError);
        HPAlignDepthErrorEntry.append(depthError);
        HPAlignRotationErrorEntry.append(rotationError);
        
        HPAlignLateralEntry.append(x);
        HPAlignDepthEntry.append(y);
        HPAlignRotationEntry.append(rot);
    }

    public void logModuleSupplyCurrents(int module, double drive, double steer) {
        if (moduleDriveSupplyCurrentEntry.containsKey(module))
            moduleDriveSupplyCurrentEntry.get(module).append(drive);
            
        if (moduleSteerSupplyCurrentEntry.containsKey(module))
            moduleSteerSupplyCurrentEntry.get(module).append(steer);
    }
    public void logModuleStatorCurrents(int module, double drive, double steer) {
        if (moduleDriveStatorCurrentEntry.containsKey(module))
            moduleDriveStatorCurrentEntry.get(module).append(drive);
            
        if (moduleSteerStatorCurrentEntry.containsKey(module))
            moduleSteerStatorCurrentEntry.get(module).append(steer);
    }
    public void logModuleCANCoderPosition(int module, double position) {
        if (moduleCanCoderPositionEntry.containsKey(module))
            moduleCanCoderPositionEntry.get(module).append(position);
    }

    public void logScoreEvent(int level, double elevator, double arm) {
        logEvent("Score L" + level + " with elevator " + elevator + ", arm " + arm, false);
    }
}
