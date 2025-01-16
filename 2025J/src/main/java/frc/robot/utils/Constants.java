// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ModuleConstants {
    public static final double kDriveMotorCurrentLimit = 40;
    public static final double kSteerMotorCurrentLimit = 40;
    public static final double kWheelDiameterInches = 4.0;

    public static final double kDriveMotorReduction = 6.12;
    public static final double kDriveEncoderPositionFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterInches))
        / kDriveMotorReduction;
    public static final double kDriveEncoderVelocityFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterInches))
        / kDriveMotorReduction;

    // TODO: Update Real Values
    public static final double kSteerMotorReduction = 0.0;

    public static final double kDriveS = 0.0;
    public static final double kDriveV = 0.0;
    public static final double kDriveA = 0.0;
    public static final double kDriveP = 0.0;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveFF = 0.0;

    public static final double kSteerS = 0.0;
    public static final double kSteerV = 0.0;
    public static final double kSteerA = 0.0;
    public static final double kSteerP = 0.0;
    public static final double kSteerI = 0.0;
    public static final double kSteerD = 0.0;
    public static final double kSteerFF = 0.0;

  }

  public static class DriveConstants {
    // TODO: Update Real Values
    public static final double kTrackWidth = 0.0;
    public static final double kWheelBase = 0.0;

    public static final double kMaxFloorSpeed = 0.0;

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

    // TODO: Update Real Values
    public static final double kFrontLeftCancoderOffset = 0.0;
    public static final double kFrontRightCancoderOffset = 0.0;
    public static final double kBackLeftCancoderOffset = 0.0;
    public static final double kBackRightCancoderOffset = 0.0;

  }

  public static class IntakeConstants {
    // TODO: Update real values
    public static final double kHPIntakeMotorCurrentLimit = 0.0;
    public static final double kHPIntakeSpeed = 0.0;
  }

  public static class ElevatorConstants {
    // TODO: Update real values
    public static final double kElevatorMotorCurrentLimit = 0.0;

    public static final double kElevatorForwardSoftLimit = 0.0;
    public static final double kElevatorReverseSoftLimit = 0.0;


    public static final double kS = 0.0; 
    public static final double kV = 0.0; 
    public static final double kA = 0.0; 
    public static final double kP = 0.0; 
    public static final double kI = 0.0; 
    public static final double kD = 0.0; 
    public static final double kFF = 0.0; 
  }
}
