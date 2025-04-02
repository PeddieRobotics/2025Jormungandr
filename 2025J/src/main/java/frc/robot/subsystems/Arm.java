package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Kraken;
import frc.robot.utils.LiveData;
import frc.robot.utils.RobotMap;
import frc.robot.utils.TunableConstant;

@SuppressWarnings("unused")
public class Arm extends SubsystemBase {

    private static Arm arm;
    private Kraken armMotor;
    private CANcoder armCANcoder;
    // private TunableConstant L1Setpoint, L2Setpoint, L3Setpoint, L4Setpoint, HPIntakeSetpoint, stowSetpoint,
    //         bargeSetpoint, algaeL1Setpoint, algaeL2Setpoint, processorSetpoint;

    private TunableConstant L1Setpoint, L2Setpoint, L3Setpoint, L4Setpoint, HPIntakeSetpoint, stowSetpoint, bargeSetpoint, algaeL1Setpoint, algaeL2Setpoint, processorSetpoint;
    private LiveData armAngle, armSetpoint, motorTemp, motorCurrent, armEncoderPosition, armCanCoderPosition;

    public Arm() {
        armCANcoder = new CANcoder(RobotMap.ARM_CANCODER_ID, RobotMap.RIO_BUS);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = ArmConstants.kArmMagnetOffset;
        armCANcoder.getConfigurator().apply(config);

        armMotor = new Kraken(RobotMap.ARM_MOTOR_ID, RobotMap.RIO_BUS);

        armMotor.setInverted(true);

        armMotor.setSupplyCurrentLimit(ArmConstants.kArmSupplyCurrentLimit);
        armMotor.setStatorCurrentLimit(ArmConstants.kArmStatorCurrentLimit);
        armMotor.setForwardTorqueCurrentLimit(ArmConstants.kArmForwardTorqueCurrentLimit);
        armMotor.setReverseTorqueCurrentLimit(ArmConstants.kArmReverseTorqueCurrentLimit);

        armMotor.setBrake();

        armMotor.setEncoder(0);
        armMotor.setFeedbackDevice(RobotMap.ARM_CANCODER_ID, FeedbackSensorSourceValue.RemoteCANcoder);
        armMotor.setRotorToSensorRatio(ArmConstants.kArmRotorToSensorRatio);
        armMotor.setSensorToMechanismRatio(ArmConstants.kArmSensortoMechanismRatio);

        // Slot 0
        armMotor.setPIDValues(ArmConstants.kSUp, ArmConstants.kVUp,
                ArmConstants.kAUp,
                ArmConstants.kPUp, ArmConstants.kIUp, ArmConstants.kDUp,
                ArmConstants.kFFUp, ArmConstants.kGUp, GravityTypeValue.Arm_Cosine, StaticFeedforwardSignValue.UseVelocitySign);

        // Slot 1
        armMotor.setPIDValuesSlot1(ArmConstants.kSDown, ArmConstants.kVDown,
                ArmConstants.kADown,
                ArmConstants.kPDown, ArmConstants.kIDown, ArmConstants.kDDown,
                ArmConstants.kFFDown, ArmConstants.kGDown, GravityTypeValue.Arm_Cosine, StaticFeedforwardSignValue.UseVelocitySign);
        
        armMotor.setMotionMagicParameters(ArmConstants.kArmMaxCruiseVelocity, ArmConstants.kArmMaxCruiseAcceleration,
                ArmConstants.kArmMaxCruiseJerk);

        armMotor.setSoftLimits(true, ArmConstants.kArmForwardSoftLimit, ArmConstants.kArmReverseSoftLimit);

        // L1Setpoint = new TunableConstant(ArmConstants.kL1Setpoint, "Arm L1Setpoint");
        // L2Setpoint = new TunableConstant(ArmConstants.kL2Setpoint, "Arm L2Setpoint");
        // L3Setpoint = new TunableConstant(ArmConstants.kL3Setpoint, "Arm L3Setpoint");
        // L4Setpoint = new TunableConstant(ArmConstants.kL4Setpoint, "Arm L4Setpoint");
        // HPIntakeSetpoint = new TunableConstant(ArmConstants.kHPIntakeSetpoint, "Arm HPIntakeSetpoint");
        // stowSetpoint = new TunableConstant(ArmConstants.kStowSetpoint, "Arm stowSetpoint");
        // bargeSetpoint = new TunableConstant(ArmConstants.kBargeSetpoint, "Arm bargeSetpoint");
        // algaeL1Setpoint = new TunableConstant(ArmConstants.kAlgaeL1Setpoint, "Arm algaeL1Setpoint");
        // algaeL2Setpoint = new TunableConstant(ArmConstants.kAlgaeL2Setpoint, "Arm algaeL2Setpoint");
        // processorSetpoint = new TunableConstant(ArmConstants.kProcessorSetpoint, "Arm processorSetpoint");

        armSetpoint = new LiveData(ArmConstants.kStowSetpoint, "Arm: Current Setpoint"); 
        armAngle = new LiveData(getArmAngleDegrees(), "Arm: Current Angle"); 
        armEncoderPosition = new LiveData(getArmMotorEncoderPosition(), "Arm: Encoder Position"); 
        armCanCoderPosition = new LiveData(getAbsoluteCANcoderPosition(), "Arm: CanCoder Position");
        

        motorTemp = new LiveData(armMotor.getMotorTemperature(), "Arm: Motor Temp"); 
        motorCurrent = new LiveData(armMotor.getSupplyCurrent(), "Arm: Motor Current");

        SmartDashboard.putBoolean("Arm: Open Loop Control", false);
    }

    /**
     * @return the existing arm instance or creates it if it doesn't exist
     */
    public static Arm getInstance() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

    /**
     * Sets armMotor speed to a designated percent output (open loop control)
     * 
     * @param speed - Percent of armMotor's speed (-1.0 to 1.0)
     */
    public void setArmPercentOutput(double percentOutput) {
        armMotor.setPercentOutput(percentOutput);
    }

    /**
     * Commands armMotor to a designated position with position voltage PID (closed
     * loop control)
     * 
     * @param position - commanded motor position (cancoder units)
     */
    public void setArmPositionVoltage(double position) {
        armSetpoint.setNumber(position);
        armMotor.setPositionVoltage(position);
    }

    /**
     * Commands armMotor to a designated position with MotionMagic voltage (closed
     * loop control)
     * 
     * @param position - commanded motor position (cancoder units)
     */
    public void setArmPositionMotionMagicVoltage(double position, int slot) {
        armSetpoint.setNumber(position);
        armMotor.setPositionMotionMagicVoltage(position, slot);
    }

    /**
     * Commands armMotor to a designated position with MotionMagic TorqueCurrentFOC
     * (closed loop control)
     * 
     * @param position - commanded motor position (cancoder units)
     */
    public void setArmPositionMotionMagicTorqueCurrentFOC(double position) {
        armSetpoint.setNumber(position);
        armMotor.setPositionMotionMagicTorqueCurrentFOC(position);
    }

    // Accessor methods

    /**
     * CANcoder reads 0 to 1
     * 
     * @return absolute CANcoder reading (rotations of CANcoder)
     */
    public double getAbsoluteCANcoderPosition() {
        return armCANcoder.getPosition().getValueAsDouble();
    }

    /**
     * @return position reading of the armMotor encoder IN MECHANISM ROTATIONS (motor encoder units)
     */
    public double getArmMotorEncoderPosition() {
        return armMotor.getPosition();
    }

    /**
     * @return velocity of armMotor encoder (rotor rotations per second)
     */
    public double getArmVelocity() {
        return armMotor.getRPS();
    }

    /**
     * @return arm motor TorqueCurrent draw (amps)
     */
    public double getMotorTorqueCurrent() {
        return armMotor.getTorqueCurrent();
    }

    /**
     * @return arm motor stator current draw (amps)
     */
    public double getMotorStatorCurrent() {
        return armMotor.getStatorCurrent();
    }

    /**
     * @return arm motor supply current draw (amps)
     */
    public double getMotorSupplyCurrent() {
        return armMotor.getSupplyCurrent();
    }

    /**
     * @return angle of arm in degrees
     */
    public double getArmAngleDegrees() {
        return getAbsoluteCANcoderPosition() * 360.0 / 2.0;
    }

    /**
     * @return setpoint position of arm (mechanism rotations)
     */
    public double getArmSetpoint() {
        return armSetpoint.getNumber();
    }

    /**
     * @param targetAngle - In degrees
     * @return returns if the difference between current and target angle is within
     *         threshold
     */
    public boolean isAtAngle(double targetAngle) {
        return Math.abs(getArmAngleDegrees() - targetAngle) < ArmConstants.kArmAngleEpsilon;
    }

    /**
     * @param targetPosition - In mechanism rotations
     * @return returns if the difference between current and target positions is within
     *         threshold
     */
    public boolean isAtPosition(double targetPosition) {
        return Math.abs(getArmMotorEncoderPosition() - targetPosition) < ArmConstants.kArmPositionEpsilon;
    }

    public double getArmMotorTemperature() {
        return armMotor.getMotorTemperature();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm: Motor Encoder Position", getArmMotorEncoderPosition());
        SmartDashboard.putNumber("Arm: CanCoder Position", getAbsoluteCANcoderPosition());

        if(SmartDashboard.getBoolean("Arm: Open Loop Control", false)){
            armMotor.setPercentOutput(DriverOI.getInstance().getRightForward() * 0.3);
        }

        armAngle.setNumber(getAbsoluteCANcoderPosition());
        armEncoderPosition.setNumber(getArmMotorEncoderPosition()); 
        armCanCoderPosition.setNumber(getAbsoluteCANcoderPosition());

        motorTemp.setNumber(armMotor.getMotorTemperature()); 
        motorCurrent.setNumber(armMotor.getSupplyCurrent()); 
    }

    @Override
    public void simulationPeriodic() {
    }

}
