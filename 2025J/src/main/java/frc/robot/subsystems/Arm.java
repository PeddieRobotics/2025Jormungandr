package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

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

        armMotor.setSupplyCurrentLimit(ArmConstants.kArmSupplyCurrentLimit);
        armMotor.setStatorCurrentLimit(ArmConstants.kArmStatorCurrentLimit);
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

    /**
     * @return the existing arm instance or creates it if it doesn't exist
     */
    public static Arm getInstance(){
        if (arm == null){
            arm = new Arm();
        }
        return arm;
    }

    /**
     * Sets armMotor speed to a designated percent output (open loop control)
     * 
     * @param speed - Percent of armMotor's speed (-1.0 to 1.0)
     */
    public void setArmPercentOutput(double percentOutput){
        armMotor.setPercentOutput(percentOutput);
    }

    /**
     * Commands armMotor to a designated position with position voltage PID (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setArmPositionVoltage(double position){
        armMotor.setPositionVoltage(position);
    }

    /**
     * Commands armMotor to a designated position with MotionMagic voltage (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setArmPositionMotionMagicVoltage(double position){
        armMotor.setPositionMotionMagicVoltage(position);
    }

    /**
     * Commands armMotor to a designated position with MotionMagic TorqueCurrentFOC (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setArmPositionMotionMagicTorqueCurrentFOC(double position){
        armMotor.setPositionMotionMagicTorqueCurrentFOC(position);
    }

    //Accessor methods

    /**
     * CANcoder reads 0 to 1
     * @return absolute CANcoder reading (rotations of CANcoder)
     */
    public double getAbsoluteCANcoderPosition() {
        return armCANcoder.getPosition().getValueAsDouble();
    }
    /**
     * @return position reading of the armMotor encoder (motor encoder units)
     */
    public double getArmPosition(){
        return armMotor.getPosition();
    }

    /**
     * @return arm motor TorqueCurrent draw (amps)
     */
    public double getMotorTorqueCurrent(){
        return armMotor.getTorqueCurrent();
    }

    /**
     * @return arm motor stator current draw (amps)
     */
    public double getMotorStatorCurrent(){
        return armMotor.getStatorCurrent();
    }
    
    /**
     * @return arm motor supply current draw (amps)
     */
    public double getMotorSupplyCurrent(){
        return armMotor.getSupplyCurrent();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

}
