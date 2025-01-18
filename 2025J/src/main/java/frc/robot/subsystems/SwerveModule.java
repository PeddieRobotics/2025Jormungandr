// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.Kraken;

public class SwerveModule extends SubsystemBase {
  
  private final CANcoder canCoder;

  private final int drivingCANId;
  private final int steerCANId;
  private final int CANCoderId;

  private final Kraken driveMotor;
  private final Kraken steerMotor;

  private SwerveModuleState desiredState;
  private double moduleAngularOffset;
  
  public SwerveModule(String canbusName, int drivingCANId, int steerCANId, int CANCoderId, double moduleAngularOffset) {
    this.drivingCANId = drivingCANId;
    this.steerCANId = steerCANId;
    this.CANCoderId = CANCoderId;
    this.moduleAngularOffset = moduleAngularOffset;

    driveMotor = new Kraken(this.drivingCANId, canbusName);
    steerMotor = new Kraken(this.steerCANId, canbusName);

    driveMotor.setInverted(true);
    steerMotor.setInverted(true);

    driveMotor.setSupplyCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);
    steerMotor.setSupplyCurrentLimit(ModuleConstants.kSteerMotorCurrentLimit);

    driveMotor.setBrake();
    steerMotor.setBrake();

    driveMotor.setClosedLoopRampRate(0.1);
    steerMotor.setClosedLoopRampRate(0.1);

    driveMotor.setEncoder(0);
    steerMotor.setEncoder(0);

    canCoder = new CANcoder(CANCoderId, canbusName);
    configureCANCoder();

    steerMotor.setContinuousOutput();
    steerMotor.setFeedbackDevice(CANCoderId, FeedbackSensorSourceValue.FusedCANcoder);
    
    driveMotor.setVelocityConversionFactor(ModuleConstants.kDriveEncoderVelocityFactor);
    steerMotor.setRotorToSensorRatio(ModuleConstants.kSteerMotorReduction);
    steerMotor.setSensorToMechanismRatio(1.0);

    driveMotor.setVelocityPIDValues(ModuleConstants.kDriveS, ModuleConstants.kDriveV, ModuleConstants.kDriveA, 
                                    ModuleConstants.kDriveP, ModuleConstants.kDriveI, ModuleConstants.kDriveD, ModuleConstants.kDriveFF);
    steerMotor.setVelocityPIDValues(ModuleConstants.kSteerS, ModuleConstants.kSteerV, ModuleConstants.kSteerA, 
                                    ModuleConstants.kSteerP, ModuleConstants.kSteerI, ModuleConstants.kSteerD, ModuleConstants.kSteerFF, StaticFeedforwardSignValue.UseClosedLoopSign);
  }

  public void configureCANCoder(){
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // Setting this to 1 makes the absolute position unsigned [0, 1)
                                                                // Setting this to 0.5 makes the absolute position signed [-0.5, 0.5)
                                                                // Setting this to 0 makes the absolute position always negative [-1, 0)
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = -moduleAngularOffset/ (2 * Math.PI);
    canCoder.getConfigurator().apply(config);
  }

  public double getCANCoderReading(){
    return  2 * Math.PI * canCoder.getAbsolutePosition().getValueAsDouble();
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(driveMotor.getMPS(), new Rotation2d(getCANCoderReading()));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveMotor.getPosition() * ModuleConstants.kDriveEncoderPositionFactor, new Rotation2d(getCANCoderReading()));
  }

  public void setDesiredState(SwerveModuleState desiredModuleState){
    desiredState = desiredModuleState;
    desiredState.optimize(new Rotation2d(getCANCoderReading()));

    double desiredVelocity = desiredState.speedMetersPerSecond;
    double desiredAngle = desiredState.angle.getRadians() / (2 * Math.PI);

    SmartDashboard.putNumber(drivingCANId + " optimized desired velocity", desiredVelocity);
    SmartDashboard.putNumber(steerCANId + " optimized desired angle", desiredAngle);
    SmartDashboard.putNumber(CANCoderId + " cancoder position", getCANCoderReading());

    driveMotor.setVelocityWithFeedForward(desiredVelocity);
    steerMotor.setPositionWithFeedForward(desiredAngle);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(CANCoderId + " CANCoder Reading", getCANCoderReading());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
