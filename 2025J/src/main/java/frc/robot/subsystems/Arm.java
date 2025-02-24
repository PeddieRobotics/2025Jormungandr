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

public class Arm extends SubsystemBase {

    private static Arm arm;
    private Kraken armMotor;
    private CANcoder armCANcoder;
    private TunableConstant L1Setpoint, L2Setpoint, L3Setpoint, L4Setpoint, HPIntakeSetpoint, stowSetpoint,
            bargeSetpoint, algaeL1Setpoint, algaeL2Setpoint, processorSetpoint;

    private LiveData armSetpoint;

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

        armMotor.setPIDValues(ArmConstants.kS, ArmConstants.kV,
                ArmConstants.kA,
                ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
                ArmConstants.kFF, ArmConstants.kG, GravityTypeValue.Arm_Cosine, StaticFeedforwardSignValue.UseClosedLoopSign);
        armMotor.setMotionMagicParameters(ArmConstants.kArmMaxCruiseVelocity, ArmConstants.kArmMaxCruiseAcceleration,
                ArmConstants.kArmMaxCruiseJerk);

        armMotor.setSoftLimits(true, ArmConstants.kArmForwardSoftLimit / 2, ArmConstants.kArmReverseSoftLimit / 2);

        L1Setpoint = new TunableConstant(ArmConstants.kL1Setpoint, "Arm L1Setpoint");
        L2Setpoint = new TunableConstant(ArmConstants.kL2Setpoint, "Arm L2Setpoint");
        L3Setpoint = new TunableConstant(ArmConstants.kL3Setpoint, "Arm L3Setpoint");
        L4Setpoint = new TunableConstant(ArmConstants.kL4Setpoint, "Arm L4Setpoint");
        HPIntakeSetpoint = new TunableConstant(ArmConstants.kHPIntakeSetpoint, "Arm HPIntakeSetpoint");
        stowSetpoint = new TunableConstant(ArmConstants.kStowSetpoint, "Arm stowSetpoint");
        bargeSetpoint = new TunableConstant(ArmConstants.kBargeSetpoint, "Arm bargeSetpoint");
        algaeL1Setpoint = new TunableConstant(ArmConstants.kAlgaeL1Setpoint, "Arm algaeL1Setpoint");
        algaeL2Setpoint = new TunableConstant(ArmConstants.kAlgaeL2Setpoint, "Arm algaeL2Setpoint");
        processorSetpoint = new TunableConstant(ArmConstants.kProcessorSetpoint, "Arm processorSetpoint");

        armSetpoint = new LiveData(stowSetpoint.get(), "Arm Current Setpoint");

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
        armSetpoint.set(position);
        armMotor.setPositionVoltage(position);
    }

    /**
     * Commands armMotor to a designated position with MotionMagic voltage (closed
     * loop control)
     * 
     * @param position - commanded motor position (cancoder units)
     */
    public void setArmPositionMotionMagicVoltage(double position) {
        armSetpoint.set(position);
        armMotor.setPositionMotionMagicVoltage(position);
    }

    /**
     * Commands armMotor to a designated position with MotionMagic TorqueCurrentFOC
     * (closed loop control)
     * 
     * @param position - commanded motor position (cancoder units)
     */
    public void setArmPositionMotionMagicTorqueCurrentFOC(double position) {
        armSetpoint.set(position);
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
     * @return position reading of the armMotor encoder (motor encoder units)
     */
    public double getArmPosition() {
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
        return getAbsoluteCANcoderPosition() * 360.0;
    }

    /**
     * @return setpoint angle of arm in degrees
     */
    public double getArmSetpoint() {
        return armSetpoint.get();
    }

    /**
     * @param targetAngle - In degrees
     * @return returns if the difference between current and target angle is within
     *         threshold
     */
    public boolean isAtAngle(double targetAngle) {
        return Math.abs(getArmAngleDegrees() - targetAngle) < ArmConstants.kArmPositionEpsilon;
    }

    public double getArmMotorTemperature() {
        return armMotor.getMotorTemperature();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm: Motor Encoder Position", getArmPosition());
        SmartDashboard.putNumber("Arm: CanCoder Position", getAbsoluteCANcoderPosition());
        // percent output on stick
        if (SmartDashboard.getBoolean("Arm: Open Loop Control", false)) {
            setArmPercentOutput(DriverOI.getInstance().getRightForward() * 0.3);
            SmartDashboard.putNumber("Arm: Open Loop Input", DriverOI.getInstance().getRightForward() * 0.3);
        }

    }

    @Override
    public void simulationPeriodic() {
    }

}
