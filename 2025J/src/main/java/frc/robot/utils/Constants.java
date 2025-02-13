// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class AutoConstants {
    // TODO: update or real values (Default was TranslationP = ThetaP = 5.0)
    public static final double kTranslationP = 0.0;
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;

    public static final double kThetaP = 0.0;
    public static final double kThetaI = 0.0;
    public static final double kThetaD = 0.0;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ModuleConstants {
    public static final double kDriveMotorSupplyCurrentLimit = 40;
    public static final double kSteerMotorSupplyCurrentLimit = 40;
    // TODO: find and update these values!
    public static final double kDriveMotorStatorCurrentLimit = 0.0;
    public static final double kSteerMotorStatorCurrentLimit = 0.0;

    public static final double kWheelDiameterInches = 3.906;
    public static final double kDriveMotorReduction = 7.13;
    public static final double kDriveEncoderPositionFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterInches))
        / kDriveMotorReduction;
    public static final double kDriveEncoderVelocityFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterInches))
        / kDriveMotorReduction;

    // TODO: Update Real Values
    public static final double kSteerMotorReduction = 18.75;

    public static final double kDriveS = 0.2;
    public static final double kDriveV = 0.12;
    public static final double kDriveA = 0.0;
    public static final double kDriveP = 0.5;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveFF = 0.0;

    public static final double kSteerS = 0.0;
    public static final double kSteerV = 0.0;
    public static final double kSteerA = 0.0;
    public static final double kSteerP = 25.0;
    public static final double kSteerI = 0.0;
    public static final double kSteerD = 0.0;
    public static final double kSteerFF = 0.0;

  }

  public static class DriveConstants {
    public static final double kTrackWidth = Units.inchesToMeters(22.75);
    public static final double kWheelBase = Units.inchesToMeters(22.75);

    public static final double kDriveRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2));

    public static final double kMaxFloorSpeed = 4;
    public static final double kMaxRotationSpeed = (3 / 2) * Math.PI;
    public static final double kMaxModuleSpeed = 4.4;
    
    public static final Translation2d[] swerveModuleLocations = {
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0),
    };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        swerveModuleLocations[0],
        swerveModuleLocations[1],
        swerveModuleLocations[2],
        swerveModuleLocations[3]);

    public static final SwerveDriveKinematics skidKinematics = new SwerveDriveKinematics(
      swerveModuleLocations[0],
      swerveModuleLocations[1],
      swerveModuleLocations[2],
      swerveModuleLocations[3]);

    // TODO: Update Real Values
    public static final double kFrontLeftCancoderOffset = 0.3684;
    public static final double kFrontRightCancoderOffset = 0.00293;
    public static final double kBackLeftCancoderOffset = 0.10083;
    public static final double kBackRightCancoderOffset = 0.2586;

    public static final double kSkidThreshold = 0.2;

  }

  public static class IntakeConstants {
    // TODO: Update real values
    public static final double kHPIntakeRollerSupplyCurrentLimit = 0.0;
    public static final double kHPIntakePivotSupplyCurrentLimit = 0.0;
    public static final double kHPIntakeSpeed = 0.0;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
  }

  public static class ElevatorConstants {
    // TODO: Update real values
    public static final double kElevatorMotorSupplyCurrentLimit = 0.0;
    public static final double kElevatorMotorStatorCurrentLimit = 0.0;

    public static final double kElevatorForwardTorqueCurrentLimit = 40;
    public static final double kElevatorReverseTorqueCurrentLimit = -40;

    public static final double kElevatorForwardSoftLimit = 0.0;
    public static final double kElevatorReverseSoftLimit = 0.0;

    // Motion Magic Parameters
    public static final double kElevatorMaxCruiseVelocity = 0.0;
    public static final double kElevatorMaxCruiseAcceleration = 0.0;
    public static final double kElevatorMaxCruiseJerk = 0.0;

    public static final double kS = 0.0; 
    public static final double kV = 0.0; 
    public static final double kA = 0.0; 
    public static final double kP = 0.0; 
    public static final double kI = 0.0; 
    public static final double kD = 0.0; 
    public static final double kFF = 0.0; 

    public static final double kElevatorHeightEpsilon = 0.0;
    public static final double kElevatorL1Height = 0.0;
    public static final double kElevatorL2Height = 0.0;
    public static final double kElevatorL3Height = 0.0;
    public static final double kElevatorL4Height = 0.0;
    
    public static final double L1Setpoint = 0.0;
    public static final double L2Setpoint = 0.0;
    public static final double L3Setpoint = 0.0;
    public static final double L4Setpoint = 0.0;

    public static final double HPIntakeSetpoint = 0.0;
    public static final double stowSetpoint = 0.0;

    public static final double bargeSetpoint = 0.0;
    public static final double algaeL1Setpoint = 0.0;
    public static final double algaeL2Setpoint = 0.0;
    public static final double processorSetpoint = 0.0;
  }


  public static final class ScoreConstants {
    //TODO: NOT DEFINED YET!!!!
    public static final double L1ScoreTimeout = 10.0; 
    public static final double L2ScoreTimeout = 10.0; //NOT DEFINED YET!!!!
    public static final double L3ScoreTimeout = 10.0; //NOT DEFINED YET!!!!
    public static final double L4ScoreTimeout = 10.0; //NOT DEFINED YET!!!!
    public static final double ProcessorTimeout = 10.0; //NOT DEFINED YET!!!!
    public static final double BargeTimeout = 10.0; //NOT DEFINED YET!!!!

    public static final double kElevatorStowPosition = 0.0;
    public static final double kElevatorHPIntakePosition = 0.0;
    public static final double kElevatorGroundIntakePosition = 0.0;
    public static final double kElevatorL1ScorePosition = 0.0;
    public static final double kElevatorL2ScorePosition = 0.0;
    public static final double kElevatorL3ScorePosition = 0.0;
    public static final double kElevatorL4ScorePosition = 0.0;
    public static final double kElevatorBargeScorePosition = 0.0;
    public static final double kElevatorProcessorScorePosition = 0.0;
    public static final double kElevatorReef1IntakePosition = 0.0;
    public static final double kElevatorReef2IntakePosition = 0.0;

    public static final double kArmStowPosition = 0.0;
    public static final double kArmHPIntakePosition = 0.0;
    public static final double kArmGroundIntakePosition = 0.0;
    public static final double kArmL1ScorePosition = 0.0;
    public static final double kArmL2ScorePosition = 0.0;
    public static final double kArmL3ScorePosition = 0.0;
    public static final double kArmL4ScorePosition = 0.0;
    public static final double kArmBargeScorePosition = 0.0;
    public static final double kArmProcessorScorePosition = 0.0;
    public static final double kArmReef1IntakePosition = 0.0;
    public static final double kArmReef2IntakePosition = 0.0;

  }
  public static final class ArmConstants {
    // TODO: Update real values
    public static final double kArmSupplyCurrentLimit = 40;
    public static final double kArmStatorCurrentLimit = 40;
    public static final double kArmForwardTorqueCurrentLimit = 40;
    public static final double kArmReverseTorqueCurrentLimit = -40;

    public static final double kArmMagnetOffset = 0.0;

    public static final double kArmReduction = 0.0;
    public static final double kArmRotorToSensorRatio = 0.0;
    public static final double kArmSensortoMechanismRatio = 0.0;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kIZone = 0.0;
    public static final double kG = 0.0;

    public static final double kArmForwardSoftLimit = 0.0;
    public static final double kArmReverseSoftLimit = 0.0;

    // Motion Magic Parameters
    public static final double kArmMaxCruiseVelocity = 0.0;
    public static final double kArmMaxCruiseAcceleration = 0.0;
    public static final double kArmMaxCruiseJerk = 0.0;

    public static final double kArmPositionEpsilon = 0.0; //TODO: define

    public static final double kArmHPIntakeAngle = 0.0; //TODO: define
    public static final double kArmL1ScoreAngle = 0.0;
    public static final double kArmL2ScoreAngle = 0.0;
    public static final double kArmL3ScoreAngle = 0.0;
    public static final double kArmL4ScoreAngle = 0.0;

    public static final double kArmL4PrepAngle = 0.0; //only l4
    public static final double L1Setpoint = 0.0;
    public static final double L2Setpoint = 0.0;
    public static final double L3Setpoint = 0.0;
    public static final double L4Setpoint = 0.0;

    public static final double HPIntakeSetpoint = 0.0;
    public static final double stowSetpoint = 0.0;

    public static final double bargeSetpoint = 0.0;
    public static final double algaeL1Setpoint = 0.0;
    public static final double algaeL2Setpoint = 0.0;
    public static final double processorSetpoint = 0.0;
  }
  public static final class AutoAlign{
    public static final InterpolatingDoubleTreeMap k1TagStdDevs = new InterpolatingDoubleTreeMap() {{
        put(1.0, 0.1);
        put(1.12, 0.5);
        put(1.33, 1.25);
        put(1.55, 1.6);
        put(1.8, 1.9);
        put(2.02, 2.7);
        put(2.21, 4.9);
        put(2.38, 7.0);
    }};
  }
  public static final class ClawConstants {
    // TODO: update real values
    public static final double kClawStatorCurrentLimit = 20.0;

    public static final double kCoralSensor1ProximityThreshold = 0.0;
    public static final double kCoralSensor2ProximityThreshold = 0.0;
    public static final double kAlgaeSensorProximityThreshold = 0.0;

    public static final double kCoralSensor1ProximityHysteresis = 0.0;
    public static final double kCoralSensor2ProximityHysteresis = 0.0;
    public static final double kAlgaeSensorProximityHysteresis = 0.0;

    public static final double kCoralSensor1SignalStrength = 0.0;
    public static final double kCoralSensor2SignalStrength = 0.0;
    public static final double kAlgaeSensorSignalStrength = 0.0;

    public static final double kClawIntakeSpeed = 0.0;
    public static final double kClawOuttakeSpeed = 0.0;
  }

  public static final class ClimberConstants {

    public static final double kClimberStatorCurrentLimit = 0;
    public static final double kClimberSupplyCurrentLimit = 0;

  }

  public static final class CameraConstants {
    // TODO: update real values
    public static final String kBackCamName = "NAME-HERE";
    public static final double kBackCamForward = 0;
    public static final double kBackCamLeftOffset = 0;
    public static final double kBackCamUpOffset = 0;
    public static final double kBackCamPanningAngle = 0;
    public static final int kBackCamPipeline = 0;

    public static final String kFrontLeftCamName = "NAME-HERE";
    public static final double kFrontLeftCamForward = 0;
    public static final double kFrontLeftCamLeftOffset = 0;
    public static final double kFrontLeftCamUpOffset = 0;
    public static final double kFrontLeftCamPanningAngle = 0;
    public static final int kFrontLeftCamPipeline = 0;

    public static final String kFrontMiddleCamName = "front-middle-cam";
    public static final double kFrontMiddleCamForward = 0;
    public static final double kFrontMiddleCamLeftOffset = 0;
    public static final double kFrontMiddleCamUpOffset = 0;
    public static final double kFrontMiddleCamPanningAngle = 0;
    public static final int kFrontMiddleCamPipeline = 0;

    public static final String kFrontRightCamName = "NAME-HERE";
    public static final double kFrontRightCamForward = 0;
    public static final double kFrontRightCamLeftOffset = 0;
    public static final double kFrontRightCamUpOffset = 0;
    public static final double kFrontRightCamPanningAngle = 0;
    public static final int kFrontRightCamPipeline = 0;

    public static final String kLeftCamName = "NAME-HERE";
    public static final double kLeftCamForward = 0;
    public static final double kLeftCamLeftOffset = 0;
    public static final double kLeftCamUpOffset = 0;
    public static final double kLeftCamPanningAngle = 0;
    public static final int kLeftCamPipeline = 0;
  }
  
  public static final class FieldConstants {
    public static final double reefCenterX = 4.5;
    public static final double reefCenterY = 4.0;
    public static final Map<Integer, Double> kAprilTagHeights = new HashMap<>() {{
      put(1, 58.5);
      put(2, 58.5);
      put(12, 58.5);
      put(13, 58.5);

      put(3, 51.125);
      put(16, 51.125);

      put(4, 74.25);
      put(5, 74.25);
      put(14, 74.25);
      put(15, 74.25);

      put(6, 12.125);
      put(7, 12.125);
      put(8, 12.125);
      put(9, 12.125);
      put(10, 12.125);
      put(11, 12.125);

      put(17, 12.125);
      put(18, 12.125);
      put(19, 12.125);
      put(20, 12.125);
      put(21, 12.125);
      put(22, 12.125);
    }};
  }
}
