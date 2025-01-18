package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.ArmConstants;

public class Arm extends SubsystemBase{

    private static Arm arm;
    private Kraken armMotor;
    private CANcoder armCANcoder;
    
    public Arm() {
        armCANcoder = new CANcoder(RobotMap.ARM_CANCODER_ID, RobotMap.CANIVORE_NAME);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = ArmConstants.kArmMagnetOffset;
        armCANcoder.getConfigurator().apply(config);

        armMotor = new Kraken(RobotMap.ARM_MOTOR_ID, RobotMap.CANIVORE_NAME);

        armMotor.setInverted(false);
        armMotor.setSupplyCurrentLimit(ArmConstants.kArmCurrentLimit);
        armMotor.setForwardTorqueCurrentLimit(ArmConstants.kArmForwardTorqueCurrentLimit);
        armMotor.setReverseTorqueCurrentLimit(ArmConstants.kArmReverseTorqueCurrentLimit);

        armMotor.setBrake();

        armMotor.setEncoder(0);
        armMotor.setFeedbackDevice(RobotMap.ARM_CANCODER_ID, FeedbackSensorSourceValue.FusedCANcoder);
        armMotor.setRotorToSensorRatio(ArmConstants.kArmRotorToSensorRatio);
        armMotor.setSensorToMechanismRatio(ArmConstants.kArmSensortoMechanismRatio);

        armMotor.setVelocityPIDValues(ArmConstants.kS, ArmConstants.kV,
                ArmConstants.kA,
                ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
                ArmConstants.kFF);
        armMotor.setMotionMagicParameters(ArmConstants.kArmMaxCruiseVelocity, ArmConstants.kArmMaxCruiseAcceleration, ArmConstants.kArmMaxCruiseJerk);

        armMotor.setSoftLimits(true, ArmConstants.kArmForwardSoftLimit, ArmConstants.kArmReverseSoftLimit);

    }

    public static Arm getInstance(){
        if (arm == null){
            arm = new Arm();
        }
        return arm;
    }

    public double getAbsoluteCANcoderPosition() {
        return armCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setArmPercentOutput(double percentOutput){
        armMotor.setMotor(percentOutput);
    }

    public void setArmPositionVoltage(double position){
        armMotor.setPosition(position);
    }

    public void setArmPositionMotionMagic(double position){
        armMotor.setPositionMotionMagic(position);
    }

    public void setMotionMagicTorqueCurrentFOC(double position){
        armMotor.setMotionMagicTorqueCurrentFOC(position);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

}
