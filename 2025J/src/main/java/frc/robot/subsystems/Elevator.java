package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.ElevatorConstants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
// import frc.robot.utils.TunableConstant;
import frc.robot.utils.LiveData;

public class Elevator extends SubsystemBase {
    private static Elevator elevator;
    private Kraken elevatorMainMotor, elevatorFollowerMotor;
    private CANcoder elevatorCANcoder;
    // private TunableConstant L1Setpoint, L2Setpoint, L3Setpoint, L4Setpoint, HPIntakeSetpoint, stowSetpoint, bargeSetpoint,
    //         algaeL1Setpoint, algaeL2Setpoint, processorSetpoint;

    private LiveData elevatorSetpoint;

    public Elevator() {
        elevatorCANcoder = new CANcoder(RobotMap.ELEVATOR_CANCODER_ID, RobotMap.CANIVORE_NAME);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // Setting this to 1 makes the absolute position
                                                                  // unsigned [0, 1)
                                                                  // Setting this to 0.5 makes the absolute position
                                                                  // signed [-0.5, 0.5)
                                                                  // Setting this to 0 makes the absolute position
                                                                  // always negative [-1, 0)
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = ElevatorConstants.kElevatorMagnetOffset;
        elevatorCANcoder.getConfigurator().apply(config); 

        elevatorMainMotor = new Kraken(RobotMap.ELEVATOR_MAIN_ID, RobotMap.CANIVORE_NAME);
        elevatorFollowerMotor = new Kraken(RobotMap.ELEVATOR_SECONDARY_ID, RobotMap.CANIVORE_NAME);

        elevatorMainMotor.setInverted(true);
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
        
        elevatorMainMotor.setEncoder(0);
        elevatorMainMotor.setFeedbackDevice(RobotMap.ELEVATOR_CANCODER_ID, FeedbackSensorSourceValue.FusedCANcoder);
        elevatorMainMotor.setRotorToSensorRatio(ElevatorConstants.kElevatorRotorToSensorRatio);
        elevatorMainMotor.setSensorToMechanismRatio(ElevatorConstants.kElevatorSensortoMechanismRatio);

        elevatorMainMotor.setPIDValues(ElevatorConstants.kS, ElevatorConstants.kV,
                ElevatorConstants.kA,
                ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
                ElevatorConstants.kFF, ElevatorConstants.kG, GravityTypeValue.Elevator_Static);

        elevatorMainMotor.setMotionMagicParameters(ElevatorConstants.kElevatorMaxCruiseVelocity,
                ElevatorConstants.kElevatorMaxCruiseAcceleration, ElevatorConstants.kElevatorMaxCruiseJerk);

        elevatorMainMotor.setSoftLimits(true, ElevatorConstants.kElevatorForwardSoftLimit,
        ElevatorConstants.kElevatorReverseSoftLimit);

        // L1Setpoint = new TunableConstant(ElevatorConstants.kL1Setpoint, "Elevator L1Setpoint");
        // L2Setpoint = new TunableConstant(ElevatorConstants.kL2Setpoint, "Elevator L2Setpoint");
        // L3Setpoint = new TunableConstant(ElevatorConstants.kL3Setpoint, "Elevator L3Setpoint");
        // L4Setpoint = new TunableConstant(ElevatorConstants.kL4Setpoint, "Elevator L4Setpoint");
        // HPIntakeSetpoint = new TunableConstant(ElevatorConstants.kHPIntakeSetpoint, "Elevator HPIntakeSetpoint");
        // stowSetpoint = new TunableConstant(ElevatorConstants.kStowSetpoint, "Elevator stowSetpoint");

        // bargeSetpoint = new TunableConstant(ElevatorConstants.kBargeSetpoint, "Elevator bargeSetpoint");
        // algaeL1Setpoint = new TunableConstant(ElevatorConstants.kAlgaeL1Setpoint, "Elevator algaeL1Setpoint");
        // algaeL2Setpoint = new TunableConstant(ElevatorConstants.kAlgaeL2Setpoint, "Elevator algaeL2Setpoint");
        // processorSetpoint = new TunableConstant(ElevatorConstants.kProcessorSetpoint, "Elevator processorSetpoint");

        elevatorSetpoint = new LiveData(ElevatorConstants.kStowSetpoint, "Elevator Current Setpoint");

        SmartDashboard.putBoolean("Elevator: Open Loop Control", false);

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

    public void setElevatorNeutralMode(){
        elevatorMainMotor.setNeutralControl();
    }

    // Accessor methods

    /**
     * @return returns cancoder reading from elevator in rotations, more accurate than just encoder
     */
    public double getElevatorCANcoderReading() {
        return elevatorCANcoder.getPosition().getValueAsDouble();
    }

    public boolean isAtPosition(double desiredPosition) {
        return Math.abs(getElevatorCANcoderReading() - desiredPosition) < ElevatorConstants.kElevatorPositionEpsilon;
    }

    public boolean isAtBottom() {
        return Math.abs(getElevatorCANcoderReading()) < ElevatorConstants.kElevatorNeutralModePositionEpsilon;
    }

    /**
     * @return position reading of the elevatorMainMotor encoder (motor encoder
     *         units)
     */
    public double getElevatorPosition() {
        return elevatorMainMotor.getPosition();
    }

    /**
     * resets encoder on elevatorMotor
     */
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

    public double getElevatorMainMotorTemperature() {
        return elevatorMainMotor.getMotorTemperature();
    }

    public double getElevatorFollowerMotorTemperature() {
        return elevatorFollowerMotor.getMotorTemperature();
    }

    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("Elevator: Open Loop Control", false)){
            setElevatorPercentOutput(DriverOI.getInstance().getRightForward() * 0.3);
        }
        SmartDashboard.putNumber("Elevator: Commanded Percent Output", DriverOI.getInstance().getForward() * 0.3);
        SmartDashboard.putNumber("Elevator: Motor Encoder Position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator: CanCoder Position", getElevatorCANcoderReading());
        SmartDashboard.putNumber("Elevator: Main Motor Supply Current", getMotorSupplyCurrent());
        SmartDashboard.putNumber("Elevator: Main Motor Stator Current", getMotorStatorCurrent());
    }

    @Override
    public void simulationPeriodic() {
    }
}
