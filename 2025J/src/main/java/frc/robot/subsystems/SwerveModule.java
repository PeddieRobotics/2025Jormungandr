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
import frc.robot.utils.Logger;

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

        driveMotor.setSupplyCurrentLimit(ModuleConstants.kDriveMotorSupplyCurrentLimit);
        steerMotor.setSupplyCurrentLimit(ModuleConstants.kSteerMotorSupplyCurrentLimit);

        driveMotor.setBrake();
        steerMotor.setCoast();

        //TODO: find these values
        //driveMotor.setStatorCurrentLimit(ModuleConstants.kDriveMotorStatorCurrentLimit);
        //steerMotor.setStatorCurrentLimit(ModuleConstants.kSteerMotorStatorCurrentLimit);

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

        driveMotor.setPIDValues(ModuleConstants.kDriveS, ModuleConstants.kDriveV, ModuleConstants.kDriveA, 
                                                                        ModuleConstants.kDriveP, ModuleConstants.kDriveI, ModuleConstants.kDriveD, ModuleConstants.kDriveFF);
        steerMotor.setPIDValues(ModuleConstants.kSteerS, ModuleConstants.kSteerV, ModuleConstants.kSteerA, 
                                                                        ModuleConstants.kSteerP, ModuleConstants.kSteerI, ModuleConstants.kSteerD, ModuleConstants.kSteerFF, StaticFeedforwardSignValue.UseClosedLoopSign);
    }

    /**
     * configures cancoder to all our settings
     */
    public void configureCANCoder(){
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // Setting this to 1 makes the absolute position unsigned [0, 1)
                                                                                                                                // Setting this to 0.5 makes the absolute position signed [-0.5, 0.5)
                                                                                                                                // Setting this to 0 makes the absolute position always negative [-1, 0)
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = moduleAngularOffset;
        canCoder.getConfigurator().apply(config);
    }

    /**
     * @return returns current cancoder reading from number of rotations to radians
     */
    public double getCANCoderReading(){
        return 2 * Math.PI * canCoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * @return returns state of swervemodule
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(driveMotor.getMPS(), new Rotation2d(getCANCoderReading()));
    }

    /**
     * @return returns angle from cancoder reading in radians
     */
    public double getAngle() {
        return getCANCoderReading();
    }

    /**
     * @return returns velocity from driveMotor encoder in meters per second
     */
    public double getVelocity() {
        return driveMotor.getMPS();
    }

    /**
     * @return returns position of swerve module
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveMotor.getPosition() * ModuleConstants.kDriveEncoderPositionFactor, new Rotation2d(getCANCoderReading()));
    }

    public double getPositionRadians(){
        return driveMotor.getPosition() / ModuleConstants.kDriveMotorReduction * 2 * Math.PI;
    }

    /**
     * sets desired state for the swervemodule state, optimizes it, and sets desired velocity and angle
     * @param desiredModuleState
     */
    public void setDesiredState(SwerveModuleState desiredModuleState){
        desiredState = desiredModuleState;
        desiredState.optimize(new Rotation2d(getCANCoderReading()));

        double desiredVelocity = desiredState.speedMetersPerSecond;
        double desiredAngle = desiredState.angle.getRadians() / (2 * Math.PI);

        SmartDashboard.putNumber(drivingCANId + " optimized desired velocity", desiredVelocity);
        SmartDashboard.putNumber(steerCANId + " optimized desired angle", desiredAngle);
        SmartDashboard.putNumber(CANCoderId + " cancoder position", getCANCoderReading());

        driveMotor.setVelocityVoltageWithFeedForward(desiredVelocity);
        steerMotor.setPositionVoltageWithFeedForward(desiredAngle);
    }

    public double getDriveMotorTemperature(){
        return driveMotor.getMotorTemperature();
    }

    public double getSteerMotorTemperature(){
        return steerMotor.getMotorTemperature();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber(CANCoderId + " CANCoder Reading", getCANCoderReading());
        SmartDashboard.putNumber(CANCoderId + "Swerve Drive Motor Current", driveMotor.getSupplyCurrent()); 
        SmartDashboard.putNumber(CANCoderId + "Swerve Steer Motor Current", steerMotor.getSupplyCurrent());
        Logger.getInstance().logModuleSupplyCurrents(CANCoderId, driveMotor.getSupplyCurrent(), steerMotor.getSupplyCurrent());
        Logger.getInstance().logModuleStatorCurrents(CANCoderId, driveMotor.getStatorCurrent(), steerMotor.getStatorCurrent());
        Logger.getInstance().logModuleCANCoderPosition(CANCoderId, getCANCoderReading());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
