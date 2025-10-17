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
        public static final double kTranslationP = 5.0;
        public static final double kTranslationI = 0.0;
        public static final double kTranslationD = 0.0;

        public static final double kThetaP = 3.0;
        public static final double kThetaI = 0.0;
        public static final double kThetaD = 0.0;
    }

    public static class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static class ModuleConstants {
        public static final double kDriveMotorSupplyCurrentLimit = 60;
        public static final double kSteerMotorSupplyCurrentLimit = 40;
        // TODO: find and update these values!
        public static final double kDriveMotorStatorCurrentLimit = 0.0;
        public static final double kSteerMotorStatorCurrentLimit = 0.0;

        public static final double kWheelDiameterInches = 3.82;
        public static final double kDriveMotorReduction = 7.13;
        public static final double kDriveEncoderPositionFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterInches))
                / kDriveMotorReduction;
        public static final double kDriveEncoderVelocityFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterInches))
                / kDriveMotorReduction;

        public static final double kSteerMotorReduction = 18.75;

        public static final double kDriveS = 0.49; 
        public static final double kDriveV = 0.11;
        public static final double kDriveA = 0.0;
        public static final double kDriveP = 0.5;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveFF = 0.0;

        public static final double kSteerS = 0.0;
        public static final double kSteerV = 0.0;
        public static final double kSteerA = 0.0;
        public static final double kSteerP = 100.0; 
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 4.0; 
        public static final double kSteerFF = 0.0;

    }

    public static class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(22.75);
        public static final double kWheelBase = Units.inchesToMeters(22.75);

        public static final double kDriveRadius = Math.sqrt(Math.pow(kTrackWidth, 2) + Math.pow(kWheelBase, 2));

        public static final double kMaxFloorSpeed = 4;
        public static final double kMaxRotationSpeed = (3 / 2) * Math.PI;
        public static final double kMaxModuleSpeed = 4.4;

        public static final Translation2d[] kSwerveModuleLocations = {
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0),
        };

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                kSwerveModuleLocations[0],
                kSwerveModuleLocations[1],
                kSwerveModuleLocations[2],
                kSwerveModuleLocations[3]);

        public static final SwerveDriveKinematics kSkidKinematics = new SwerveDriveKinematics(
                kSwerveModuleLocations[0],
                kSwerveModuleLocations[1],
                kSwerveModuleLocations[2],
                kSwerveModuleLocations[3]);

        // Steps to doing offsets for swerves:
        // 1. remove offsets (set all to 0) and then deploy code
        // 2. spin all modules so that bevel gears face left relative to robot (shooter
        // in front)
        // 3. read the cancoder values from dashboard, and negate those values
        // // offsets (check robotmap for ids) and put them here

        // public static final double kFrontLeftMagnetOffset = 0.238525;
        // public static final double kFrontRightMagnetOffset = 0.157471;
        // public static final double kBackLeftMagnetOffset = 0.496094;
        // public static final double kBackRightMagnetOffset = 0.130127;

        public static final double kFrontLeftMagnetOffset = 0.232666;
        public static final double kFrontRightMagnetOffset = 0.154541;
        public static final double kBackLeftMagnetOffset = 0.496338;
        public static final double kBackRightMagnetOffset = 0.129883;

        public static final double kSkidThreshold = 0.2;
        public static final double kDrivingDeadband = 0.05;

        public static final double kCardinalDirectionSpeedScale = 0.1;

    }

    public static class IntakeConstants {
        public static final double kHPIntakeActuatorExtendedPostion = 1.0;
        public static final double kHPIntakeActuatorRetractedPostion = 0.0;
    }

    public static class ElevatorConstants {
        public static final double kElevatorMotorSupplyCurrentLimit = 60.0;
        public static final double kElevatorMotorStatorCurrentLimit = 60.0;

        public static final double kElevatorForwardTorqueCurrentLimit = 40;
        public static final double kElevatorReverseTorqueCurrentLimit = -40;

        public static final double kElevatorForwardSoftLimit = 12.9;
        public static final double kElevatorReverseSoftLimit = 0.0;

        public static final double kElevatorReduction = 5.250  / 1.0;
        public static final double kElevatorRotorToSensorRatio = kElevatorReduction;
        public static final double kElevatorSensortoMechanismRatio = 1.0;

        public static final double kElevatorMagnetOffset = 0.0;

        // Motion Magic Parameters
        // public static final double kElevatorMaxCruiseVelocity = 14.0;
        // public static final double kElevatorMaxCruiseAcceleration = 40.0;
        // public static final double kElevatorMaxCruiseJerk = 400.0;
        // TODO: REDO MOTION MAGIC PARAMETERS
        public static final double kElevatorMaxCruiseVelocity = 18.0;
        public static final double kElevatorMaxCruiseAcceleration = 80.0;
        public static final double kElevatorMaxCruiseJerk = 800.0;

        // 7:1 gear ratio | voltage motion magic
        // public static final double kS = 0.085;
        // public static final double kV = 0.0;
        // public static final double kA = 0.0;
        // public static final double kP = 40.0;
        // public static final double kI = 0.0;
        // public static final double kD = 2.0;
        // public static final double kG = 0.255;
        // public static final double kFF = 0.0;

        // public static final double kS = 0.19;
        // public static final double kV = 0.0;
        // public static final double kA = 0.0;
        // public static final double kP = 45.0;
        // public static final double kI = 0.0;
        // public static final double kD = 0.5;
        // public static final double kG = 0.3;
        // public static final double kFF = 0.0;

        public static final double kS = 0.17;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double kP = 40.0;
        public static final double kI = 0.0;
        public static final double kD = 0.7;
        public static final double kG = 0.36;
        public static final double kFF = 0.0;

        public static final double kElevatorPositionEpsilon = 0.03;
        public static final double kElevatorPositionEpsilonAuto = 0.2;
        public static final double kElevatorNeutralModePositionEpsilon = 0.1;

        public static final double kMinHomeTime = 0.25;
        public static final double kHomeSpeed = -0.1;
        public static final double kHomingSpeedThreshold = 0.01;
    }

    public static final class ScoreConstants {
        // TODO: NOT DEFINED YET!!!!
        public static final double kL1ScoreTimeout = 2.0;
        public static final double kL2ScoreTimeout = 1.0; 
        public static final double kL3ScoreTimeout = 1.0; 
        public static final double kL4ScoreTimeout = 1.0; 
        public static final double kProcessorTimeout = 1.0; 
        public static final double kBargeTimeout = 0.7; // 1.0;

        public static final double kElevatorStowPosition = 0.0;
        public static final double kElevatorHPIntakePosition = 0.0;
        public static final double kElevatorGroundIntakePosition = 0.77;
        public static final double kElevatorLollipopIntakePosition = 0;
        public static final double kElevatorL1ScorePosition = 3.65 - 1.5 + 0.05;
        // public static final double kElevatorL1ScorePosition = 1.75;;
        public static final double kElevatorL2ScorePosition = 3.65; //3.8;
        public static final double kElevatorL3ScorePosition = 7.0;
        public static final double kElevatorPrestagePosition = 3.3;

        // Seneca Height
        // public static final double kElevatorL4ScorePosition = 12.45; 
        // Lab Height
        public static final double kElevatorL4ScorePosition = 12.15; 

        public static final double kElevatorBargePrestagePosition = 6.2;
        public static final double kElevatorBargeScorePosition = 12.85;
        public static final double kElevatorProcessorScorePosition = 0.0;
        public static final double kElevatorReef1IntakePosition = 2.0;
        public static final double kElevatorReef2IntakePosition = 5.25;
        public static final double kElevatorClimbPosition = 0.0;

        public static final double kArmStowPosition = 0.25;
        public static final double kArmHPIntakePosition = 0.25;
        public static final double kArmGroundIntakePosition = 0.0;
        public static final double kArmLollipopIntakePosition = 0.10;
        public static final double kArmL1ScorePosition = 0.25;
        // public static final double kArmL1ScorePosition = 0.0;
        public static final double kArmL2ScorePosition = 0.25;
        public static final double kArmL3ScorePosition = 0.25;
        public static final double kArmL4ScorePosition = 0.25;
        public static final double kArmBargeScorePosition = 0.25;
        public static final double kArmProcessorScorePosition = 0.09;
        public static final double kArmReef1IntakePosition = 0.17;
        public static final double kArmReef2IntakePosition = 0.17;
        public static final double kArmClimbPosition = 0.09; 

    }

    public static final class ArmConstants {
        public static final double kArmSupplyCurrentLimit = 40;
        public static final double kArmStatorCurrentLimit = 40;
        public static final double kArmForwardTorqueCurrentLimit = 40;
        public static final double kArmReverseTorqueCurrentLimit = -40;

        public static final double kArmReduction = 114.288 / 1.0;
        public static final double kArmRotorToSensorRatio = kArmReduction / 0.96;
        public static final double kArmSensortoMechanismRatio = 0.96;

        public static final double kArmMagnetOffset = -0.857422  + 0.25 * kArmSensortoMechanismRatio;
        // public static final double kArmMagnetOffset = 0.0;

        // public static final double kS = 0.3;
        // public static final double kV = 0.0;
        // public static final double kA = 0.0;
        // public static final double kP = 100.0;
        // public static final double kI = 0.0;
        // public static final double kD = 5.0;
        // public static final double kFF = 0.0;
        // public static final double kIZone = 0.0;
        // public static final double kG = 0.39;

        public static final int kSlotUp = 0;
        public static final double kSUp = 0.0;
        public static final double kVUp = 0.0;
        public static final double kAUp = 0.0;
        public static final double kPUp = 60.0;
        public static final double kIUp = 0.0;
        public static final double kDUp = 2.0;
        public static final double kFFUp = 0.0;
        public static final double kIZoneUp = 0.0;
        public static final double kGUp = 0.37;

        public static final int kSlotDown = 1;
        public static final double kSDown = 0.0;
        public static final double kVDown = 0.0;
        public static final double kADown = 0.0;
        public static final double kPDown = 40.0;
        public static final double kIDown = 0.0;
        public static final double kDDown = 1.0;
        public static final double kFFDown = 0.0;
        public static final double kIZoneDown = 0.0;
        public static final double kGDown = 0.37;

        public static final double kArmForwardSoftLimit = 0.25;
        public static final double kArmReverseSoftLimit = 0.0;

        // Motion Magic Parameters
        public static final double kArmMaxCruiseVelocity = 2.0;
        public static final double kArmMaxCruiseAcceleration = 4.0;
        public static final double kArmMaxCruiseJerk = 40.0;

        public static final double kArmAngleEpsilon = 0.0; // TODO: define
        public static final double kArmPositionEpsilon = 0.02;

        public static final double kArmHPIntakeAngle = 0.0; // TODO: define
        public static final double kArmL1ScoreAngle = 0.0;
        public static final double kArmL2ScoreAngle = 0.0;
        public static final double kArmL3ScoreAngle = 0.0;
        public static final double kArmL4ScoreAngle = 0.0;

        public static final double kArmL4PrepAngle = 0.0; // only l4
        public static final double kL1Setpoint = 0.0;
        public static final double kL2Setpoint = 0.0;
        public static final double kL3Setpoint = 0.0;
        public static final double kL4Setpoint = 0.0;

        public static final double kHPIntakeSetpoint = 0.0;
        public static final double kStowSetpoint = 0.0;

        public static final double kBargeSetpoint = 0.0;
        public static final double kAlgaeL1Setpoint = 0.0;
        public static final double kAlgaeL2Setpoint = 0.0;
        public static final double kProcessorSetpoint = 0.0;
    }

    public static final class AlignmentConstants {
        public static final Map<Integer, Double> kReefDesiredAngle = new HashMap<>() {
            {
                // red side
                put(6, 120.0);
                put(7, 180.0);
                put(8, -120.0);
                put(9, -60.0);
                put(10, 0.0);
                put(11, 60.0);
    
                // blue side, same angles but opposite
                put(17, 60.0);
                put(18, 0.0);
                put(19, -60.0);
                put(20, -120.0);
                put(21, 180.0);
                put(22, 120.0);
            }
        };

        public static final Map<Integer, Integer> kBlueToRedReefTag = new HashMap<>() {
            {
                put(17, 8);
                put(18, 7);
                put(19, 6);
                put(20, 11);
                put(21, 10);
                put(22, 9);
            }
        };

        public enum AlignmentDestination {
            LEFT, MIDDLE, RIGHT
        };

        public static final class ReefAlign {
            public static final double kTranslateThreshold = 0.03;
            public static final double kTranslateThresholdAuto = 0.035;
            public static final double kTranslateThresholdL2L3 = 0.065;
            public static final double kTranslateSetpoint = 0;

            public static final double kAutoDepthP = 2.625; // went from 2.9
            public static final double kDepthP = 2.8;
            public static final double kDepthI = 0;
            public static final double kDepthD = 0;
            public static final double kDepthFF = 0;

            public static final double kDepthThreshold = 0.03;
            public static final double kDepthCloseThreshold = 0.45;
            public static final double kDepthCloseAtL4Threshold = 0.05;
            public static final double kDepthL3PrestageThreshold = 0.6;
            public static final double kDepthL4PrestageThreshold = 0.9;
            public static final double kDepthL4AutoPrestageThreshold = 1.2; // 0.9 definitely works however
            public static final double kDepthL4DaisyAutoPrestageThreshold = 1.55;

            public static final double kAutoLateralP = 2.7;
            public static final double kLateralP = 2.85;
            public static final double kLateralI = 0;
            public static final double kLateralD = 0;
            public static final double kLateralFF = 0;

            public static final double kLateralThreshold = 0.03;
            public static final double kLateralCloseThreshold = 0.15;
            public static final double kLateralCloseAtL4Threshold = 0.05;
            public static final double kLateralL3PrestageThreshold = 0.4;
            public static final double kLateralL4PrestageThreshold = 0.4;
            public static final double kLateralL4AutoPrestageThreshold = 0.5;
            public static final double kLateralL4DaisyAutoPrestageThreshold = 0.8;

            public static final double kDepthReefAlgaeThreshold = 0.9;
            public static final double kLateralReefAlgaeThreshold = 0.5;

            public static final double kRotationP = 0.08;
            public static final double kRotationI = 0;
            public static final double kRotationD = 0;
            public static final double kRotationFF = 0;
            public static final double kRotationThreshold = 0.5;
            public static final double kRotationLowerP = 0.06;
            public static final double kRotationUseLowerPThreshold = 1.5;
            
            // center of robot distance to tag -- back (+ = back, - = forwards)
            public static final double kTagBackMagnitude = 0.45; // from 0.45
                                                                 //
            public static final double kAutoTagBackMagnitude = 0.47; // from 0.45
            public static final double kAutoCloseTagBackMagnitude = 0.46; // from 0.45

            // center of robot distance to tag -- left (+ = left, - = right)
            // public static final double kLeftOffset = 0.199; // 0.1896;
            // public static final double kMiddleOffset = 0.01; //0.02;   
            // public static final double kRightOffset = -0.135; // -0.1651;

            // MONTGOMERY
            public static final double kLeftOffset = 0.216;
            public static final double kMiddleOffset = 0.01;   
            public static final double kRightOffset = -0.150;

            public static final double kL1TagBackMagnitude = 0.43;
            public static final double kL1LeftOffset = 0.3084;
            public static final double kL1RightOffset = -0.135 - 0.1194;
            
            public static final double kMaxSpeed = 3.0;

            public static final double kMaxScoringSpeed = 1.0;
        }

        public static final class HPAlign {
            public static final double kTranslateP = 3.0;
            public static final double kTranslateI = 0;
            public static final double kTranslateD = 0;
            public static final double kTranslateFF = 0;

            public static final double kTranslateThreshold = 0.02;
            public static final double kAutoTranslateThreshold = 0.05;
            public static final double kTranslateSetpoint = 0;

            public static final double kDepthThreshold = 0.08;
            public static final double kLateralThreshold = 0.02;

            public static final double kDepthP = 2.95;
            public static final double kDepthI = 0;
            public static final double kDepthD = 0;
            public static final double kDepthFF = 0;

            public static final double kLateralP = 3.1;
            public static final double kLateralI = 0;
            public static final double kLateralD = 0;
            public static final double kLateralFF = 0;

            public static final double kRotationP = 0.08;
            public static final double kRotationI = 0;
            public static final double kRotationD = 0;
            public static final double kRotationFF = 0;
            public static final double kRotationThreshold = 0.8;
            public static final double kRotationLowerP = 0.06;
            public static final double kRotationUseLowerPThreshold = 1.5;
            
            // TODO
            public static final double kBackOffset = 0.44;
            public static final double kAutoBackOffset = 0.44;
            public static final double kLateralOffset = -0.03;

            public static final double kAutoRightLateralOffset = 0.0;
            public static final double kAutoLeftLateralOffset = 0.0;
            
            public static final double kMaxSpeed = 4.0;
        }
            
        public static final double kDefaultToClosestDistance = 1.5;

        public static final double rotationErrorEpsilon = 0.0;
        public static final double xErrorEpsilon = 0.0;
        public static final double yErrorEpsilon = 0.0;

        public static final double kBadHexagonSize = 0.9;
        public static final double kInsideBadAngleTolerance = 40.0;
        public static final double kOutsideBadAngleTolerance = 55.0;
        
        public static final double kL1PressSpeed = -0.3;
        public static final double kL1StrafingPressSpeed = -0.2;
        public static final double kL1StrafeSpeed = 0.9;

        public static final double kL1PressTime = 0.7;
        public static final double kL1StrafeBeforeScoreTime = 0.00;
    }

    public static final class ClawConstants {
        public static final double kClawCoralSupplyCurrentLimit = 30.0;
        public static final double kClawCoralStatorCurrentLimit = 30.0;

        public static final double kClawAlgaeSupplyCurrentLimit = 60.0;
        public static final double kClawAlgaeStatorCurrentLimit = 60.0;

        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.0;

        public static final double kTopSensorProximityThreshold = 0.127;
        public static final double kBottomSensorProximityThreshold = 0.0;
        public static final double kAlgaeSensorProximityThreshold = 0.0;

        public static final double kTopSensorProximityHysteresis = 0.0254;
        public static final double kBottomSensorProximityHysteresis = 0.0;
        public static final double kAlgaeSensorProximityHysteresis = 0.0;

        public static final double kTopSensorSignalStrength = 2500.0;
        public static final double kBottomSensorSignalStrength = 0.0;
        public static final double kAlgaeSensorSignalStrength = 0.0;

        public static final double kTopSensorFovCenterX = 0.0;
        public static final double kBottomSensorFovCenterX = 0.0;
        public static final double kAlgaeSensorFovCenterX = 0.0;

        public static final double kTopSensorFovCenterY = 27.0;
        public static final double kBottomSensorFovCenterY = 0.0;
        public static final double kAlgaeSensorFovCenterY = 0.0;

        public static final double kTopSensorFovRangeX = 27.0;
        public static final double kBottomSensorFovRangeX = 14.0;
        public static final double kAlgaeSensorFovRangeX = 27.0;

        public static final double kTopSensorFovRangeY = 7.0;
        public static final double kBottomSensorFovRangeY = 14.0;
        public static final double kAlgaeSensorFovRangeY = 27.0;

        public static final double kCoralIntakeSpeed = 0.65;
        public static final double kCoralOuttakeSpeed = 0.75;
        public static final double kCoralSlowIntake = 0.3;
        // public static final double kCoralL1OuttakeSpeed = 0.4;
        public static final double kCoralL1OuttakeSpeed = 0.25;

        public static final double kAlgaeIntakeSpeed = 1.0;
        public static final double kAlgaeOuttakeSpeed = -1.0;

        public static final double kAlgaeHoldSpeed = 1.0;

        public static final double kCoralPositionIncrement = 3.0;
    }

    public static final class ClimberConstants {

        public static final double kClimberStatorCurrentLimit = 40.0;
        public static final double kClimberSupplyCurrentLimit = 40.0;

        public static final double kClimberReduction = 120.0 / 1.0;

        //TODO: set values
        public static final double kClimberRetractedPosition = 60.0;
        public static final double kClimberDeployedPosition = 211.0;
        public static final double kClimberRetractedPercentOutput = -0.5;
        public static final double kClimberDeployedPercentOutput = 1;

    }

    public static final class CameraConstants {
        // 3: 10.58.95.14
        public static final String kBackCamName = "limelight-back";

        // 3G: 10.58.95.11
        public static final String kFrontLeftCamName = "limelight-left";

        // 3: 10.58.95.12
        public static final String kClimberCamName = "limelight-climber";

        // 3G: 10.58.95.13
        public static final String kFrontRightCamName = "limelight-right";

        public static final InterpolatingDoubleTreeMap k1TagStdDevs = new InterpolatingDoubleTreeMap() {
            {
                put(0.0, 0.1);
                put(0.4, 0.1);
                put(0.53, 0.15);
                put(0.64, 0.25);
                put(0.74, 0.3);
                put(0.8, 0.4);
                put(0.85, 0.5);
                put(0.89, 0.55);
                put(0.93, 0.7);
                put(0.94, 1.2);
                put(1.0, 1.4);
                put(1.1, 1.6);
                put(1.25, 2.5);
            }
        };

        public static final InterpolatingDoubleTreeMap k2TagStdDevs = new InterpolatingDoubleTreeMap() {
            {
                put(0.0, 0.3);
                put(1.5, 0.5);
                put(2.5, 0.7);
                put(3.5, 0.9);
                // put(2.0, 0.2);
                // put(2.5, 0.3);
                // put(3.0, 0.5);
            }
        };
    }

    public static final class FieldConstants {
        public static final double kReefCenterXBlue = 4.5;
        public static final double kReefCenterYBlue = 4.0;
        public static final double kReefCenterXRed = 13.063;
        public static final double kReefCenterYRed = 4.0;
        public static final Map<Integer, Double> kAprilTagHeights = new HashMap<>() {
            {
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
            }
        };

        public static final double kBargeDesiredAngle = 0;
        public static final double kBargeDesiredAngleOpponent = 180;
    }
}
