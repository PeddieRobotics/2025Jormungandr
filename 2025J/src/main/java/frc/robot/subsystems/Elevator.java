package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.ElevatorConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.TunableConstant;
import frc.robot.utils.LiveData;

public class Elevator extends SubsystemBase {
    private static Elevator elevator;
    private Kraken elevatorMainMotor, elevatorFollowerMotor;
    private CANcoder elevatorCANcoder;
    private TunableConstant kP, kS, kV, kI, kD, kFF, kA,
            kElevatorMaxCruiseVelocity, kElevatorMaxCruiseAcceleration, kElevatorMaxCruiseJerk,
            kElevatorForwardSoftLimit, kElevatorReverseSoftLimit,
            L1Setpoint, L2Setpoint, L3Setpoint, L4Setpoint, HPIntakeSetpoint, stowSetpoint, bargeSetpoint,
            algaeL1Setpoint, algaeL2Setpoint, processorSetpoint;

    private LiveData elevatorSetpoint;

    public Elevator() {
        elevatorMainMotor = new Kraken(RobotMap.ELEVATOR_MAIN_ID, RobotMap.CANIVORE_NAME);
        elevatorFollowerMotor = new Kraken(RobotMap.ELEVATOR_SECONDARY_ID, RobotMap.CANIVORE_NAME);

        elevatorMainMotor.setInverted(false); // TODO: confirm direction
        elevatorFollowerMotor.setFollower(RobotMap.ELEVATOR_MAIN_ID, true);

        elevatorMainMotor.setSupplyCurrentLimit(ElevatorConstants.kElevatorMotorSupplyCurrentLimit);
        elevatorFollowerMotor.setSupplyCurrentLimit(ElevatorConstants.kElevatorMotorSupplyCurrentLimit);

        elevatorMainMotor.setStatorCurrentLimit(ElevatorConstants.kElevatorMotorStatorCurrentLimit);
        elevatorFollowerMotor.setStatorCurrentLimit(ElevatorConstants.kElevatorMotorStatorCurrentLimit);

        elevatorMainMotor.setForwardTorqueCurrentLimit(ArmConstants.kArmForwardTorqueCurrentLimit);
        elevatorMainMotor.setReverseTorqueCurrentLimit(ArmConstants.kArmReverseTorqueCurrentLimit);
        elevatorFollowerMotor.setForwardTorqueCurrentLimit(ArmConstants.kArmForwardTorqueCurrentLimit);
        elevatorFollowerMotor.setReverseTorqueCurrentLimit(ArmConstants.kArmReverseTorqueCurrentLimit);

        elevatorMainMotor.setBrake();
        elevatorFollowerMotor.setBrake();

        elevatorMainMotor.setPIDValues(ElevatorConstants.kS, ElevatorConstants.kV,
                ElevatorConstants.kA,
                ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
                ElevatorConstants.kFF);
        elevatorFollowerMotor.setPIDValues(ElevatorConstants.kS, ElevatorConstants.kV,
                ElevatorConstants.kA,
                ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
                ElevatorConstants.kFF);

        elevatorMainMotor.setMotionMagicParameters(ElevatorConstants.kElevatorMaxCruiseVelocity,
                ElevatorConstants.kElevatorMaxCruiseAcceleration, ElevatorConstants.kElevatorMaxCruiseJerk);
        elevatorFollowerMotor.setMotionMagicParameters(ElevatorConstants.kElevatorMaxCruiseVelocity,
                ElevatorConstants.kElevatorMaxCruiseAcceleration, ElevatorConstants.kElevatorMaxCruiseJerk);

        elevatorMainMotor.setSoftLimits(true, ElevatorConstants.kElevatorForwardSoftLimit,
                ElevatorConstants.kElevatorReverseSoftLimit);
        elevatorFollowerMotor.setSoftLimits(true, ElevatorConstants.kElevatorForwardSoftLimit,
                ElevatorConstants.kElevatorReverseSoftLimit);

        elevatorCANcoder = new CANcoder(RobotMap.ELEVATOR_CANCODER_ID);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // Setting this to 1 makes the absolute position unsigned [0, 1)
                                                                // Setting this to 0.5 makes the absolute position signed [-0.5, 0.5)
                                                                // Setting this to 0 makes the absolute position always negative [-1, 0)
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = ElevatorConstants.kElevatorMagnetOffset;
        elevatorCANcoder.getConfigurator().apply(config); 

        elevatorMainMotor.setFeedbackDevice(RobotMap.ELEVATOR_CANCODER_ID, FeedbackSensorSourceValue.FusedCANcoder);

        kP = new TunableConstant(ElevatorConstants.kP, "Elevator kP");
        kI = new TunableConstant(ElevatorConstants.kI, "Elevator kI");
        kD = new TunableConstant(ElevatorConstants.kD, "Elevator kD");
        kA = new TunableConstant(ElevatorConstants.kA, "Elevator A");
        kS = new TunableConstant(ElevatorConstants.kS, "Elevator kS");
        kV = new TunableConstant(ElevatorConstants.kV, "Elevator kV");
        kFF = new TunableConstant(ElevatorConstants.kFF, "Elevator kFF");
        kElevatorMaxCruiseVelocity = new TunableConstant(ElevatorConstants.kElevatorMaxCruiseVelocity,
                "Elevator kElevatorMaxCruiseVelocity");
        kElevatorReverseSoftLimit = new TunableConstant(ElevatorConstants.kElevatorReverseSoftLimit,
                "Elevator kElevatorReverseSoftLimit");
        kElevatorForwardSoftLimit = new TunableConstant(ElevatorConstants.kElevatorForwardSoftLimit,
                "Elevator kElevatorForwardSoftLimit");
        kElevatorMaxCruiseJerk = new TunableConstant(ElevatorConstants.kElevatorMaxCruiseJerk,
                "Elevator kElevatorMaxCruiseJerk");
        kElevatorMaxCruiseAcceleration = new TunableConstant(ElevatorConstants.kElevatorMaxCruiseAcceleration,
                "Elevator kElevatorMaxCruiseAcceleration");
        L1Setpoint = new TunableConstant(ElevatorConstants.L1Setpoint, "Elevator L1Setpoint");
        L2Setpoint = new TunableConstant(ElevatorConstants.L2Setpoint, "Elevator L2Setpoint");
        L3Setpoint = new TunableConstant(ElevatorConstants.L3Setpoint, "Elevator L3Setpoint");
        L4Setpoint = new TunableConstant(ElevatorConstants.L4Setpoint, "Elevator L4Setpoint");
        HPIntakeSetpoint = new TunableConstant(ElevatorConstants.HPIntakeSetpoint, "Elevator HPIntakeSetpoint");
        stowSetpoint = new TunableConstant(ElevatorConstants.stowSetpoint, "Elevator stowSetpoint");

        bargeSetpoint = new TunableConstant(ElevatorConstants.bargeSetpoint, "Elevator bargeSetpoint");
        algaeL1Setpoint = new TunableConstant(ElevatorConstants.algaeL1Setpoint, "Elevator algaeL1Setpoint");
        algaeL2Setpoint = new TunableConstant(ElevatorConstants.algaeL2Setpoint, "Elevator algaeL2Setpoint");
        processorSetpoint = new TunableConstant(ElevatorConstants.processorSetpoint, "Elevator processorSetpoint");

        elevatorSetpoint = new LiveData(ElevatorConstants.stowSetpoint, "Elevator Current Setpoint");
    }

    /**
     * @return the existing elevator instance or creates it if it doesn't exist
     */
    public static Elevator getInstance() {
        if (elevator == null) {
            elevator = new Elevator();
        }
        return elevator;
    }

    /**
     * Sets elevatorMainMotor speed to a designated percent output (open loop
     * control)
     * 
     * @param speed - Percent of elevatorMainMotor's speed (-1.0 to 1.0)
     */
    public void setElevatorPercentOutput(double speed) {
        elevatorMainMotor.setPercentOutput(speed);
    }

    /**
     * Stops elevatorMainMotor by setting the speed to 0
     */
    public void stopElevator() {
        elevatorMainMotor.setPercentOutput(0);
    }

    /**
     * Commands elevatorMainMotor to a designated position with position voltage PID
     * (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setElevatorPositionVoltage(double position) {
        elevatorSetpoint.set(position);
        elevatorMainMotor.setPositionVoltage(position);
    }

    /**
     * Commands elevatorMainMotor to a designated position with MotionMagic voltage
     * (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setElevatorPositionMotionMagicVoltage(double position) {
        elevatorSetpoint.set(position);
        elevatorMainMotor.setPositionMotionMagicVoltage(position);
    }

    /**
     * Commands elevatorMainMotor to a designated position with MotionMagic
     * TorqueCurrentFOC (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setElevatorPositionMotionMagicTorqueCurrentFOC(double position) {
        elevatorSetpoint.set(position);
        elevatorMainMotor.setPositionMotionMagicTorqueCurrentFOC(position);
    }

    // Accessor methods

    public double getElevatorCANcoderReading() {
        return elevatorCANcoder.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double desiredPosition) {
        return Math.abs(getElevatorCANcoderReading() - desiredPosition) < ElevatorConstants.kElevatorPositionEpsilon;
    }

    /**
     * @return position reading of the elevatorMainMotor encoder (motor encoder
     *         units)
     */
    public double getElevatorPosition() {
        return elevatorMainMotor.getPosition();
    }

    public void resetElevatorPosition() {
        elevatorMainMotor.resetEncoder();
    }

    /**
     * @return elevator motor TorqueCurrent draw (amps)
     */
    public double getMotorTorqueCurrent() {
        return elevatorMainMotor.getTorqueCurrent();
    }

    /**
     * @return elevator motor stator current draw (amps)
     */
    public double getMotorStatorCurrent() {
        return elevatorMainMotor.getStatorCurrent();
    }

    /**
     * @return elevator motor supply current draw (amps)
     */
    public double getMotorSupplyCurrent() {
        return elevatorMainMotor.getSupplyCurrent();
    }

    /**
     * @return elevator motor velocity (rotations per second)
     */
    public double getElevatorVelocity(){
        return elevatorMainMotor.getRPS();
    }

    /**
     * @return elevator motor setpoint (motor encoder units)
     */
    public double getElevatorSetpoint(){
        return elevatorSetpoint.get();
    }

    @Override
    public void periodic() {
        elevatorMainMotor.setPIDValues(kS.get(), kV.get(),
                kA.get(),
                kP.get(), kI.get(), kD.get(),
                kFF.get());
        elevatorFollowerMotor.setPIDValues(kS.get(), kV.get(),
                kA.get(),
                kP.get(), kI.get(), kD.get(),
                kFF.get());

        elevatorMainMotor.setMotionMagicParameters(kElevatorMaxCruiseVelocity.get(),
                kElevatorMaxCruiseAcceleration.get(), kElevatorMaxCruiseJerk.get());
        elevatorFollowerMotor.setMotionMagicParameters(kElevatorMaxCruiseVelocity.get(),
                kElevatorMaxCruiseAcceleration.get(), kElevatorMaxCruiseJerk.get());

        elevatorMainMotor.setSoftLimits(true, kElevatorForwardSoftLimit.get(), kElevatorReverseSoftLimit.get());
        elevatorFollowerMotor.setSoftLimits(true, kElevatorForwardSoftLimit.get(), kElevatorReverseSoftLimit.get());
    }

    @Override
    public void simulationPeriodic() {
    }
}
