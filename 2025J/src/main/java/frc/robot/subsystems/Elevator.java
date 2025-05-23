package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DriverStation;
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

    private double globalOffset = 0;


    @SuppressWarnings("unused")
    private LiveData elevatorEncoderPosition, elevatorCanCoderPosition, elevatorSetpoint, mainMotorTemp, mainMotorStatorCurrent, mainMotorSupplyCurrent,
            followerMotorTemp, bottomSensorData, elevatorHeight, canCoderVelocity;

    public Elevator() {
        elevatorCANcoder = new CANcoder(RobotMap.ELEVATOR_CANCODER_ID, RobotMap.CANIVORE_BUS);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; // Setting this to 1 makes the absolute position
                                                                  // unsigned [0, 1)
                                                                  // Setting this to 0.5 makes the absolute position
                                                                  // signed [-0.5, 0.5)
                                                                  // Setting this to 0 makes the absolute position
                                                                  // always negative [-1, 0)
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = ElevatorConstants.kElevatorMagnetOffset;
        elevatorCANcoder.getConfigurator().apply(config); 

        elevatorCANcoder.setPosition(0.0);

        elevatorMainMotor = new Kraken(RobotMap.ELEVATOR_MAIN_ID, RobotMap.CANIVORE_BUS);
        elevatorFollowerMotor = new Kraken(RobotMap.ELEVATOR_SECONDARY_ID, RobotMap.CANIVORE_BUS);

        elevatorMainMotor.setInverted(false);
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
                ElevatorConstants.kFF, ElevatorConstants.kG, GravityTypeValue.Elevator_Static, StaticFeedforwardSignValue.UseVelocitySign);

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

        elevatorSetpoint = new LiveData(0.0, "Elevator: Current Setpoint");
        elevatorEncoderPosition = new LiveData(elevatorMainMotor.getPosition(), "Elevator: Current Encoder Position");
        elevatorCanCoderPosition = new LiveData(getElevatorCANcoderPosition(), "Elevator: Current Cancoder Position"); 

        mainMotorTemp = new LiveData(elevatorMainMotor.getMotorTemperature(), "Elevator: Main Motor Temp");
        followerMotorTemp = new LiveData(elevatorFollowerMotor.getMotorTemperature(), "Elevator: Follower Motor Temp");

        mainMotorSupplyCurrent = new LiveData(elevatorMainMotor.getSupplyCurrent(), "Elevator: Main Motor Supply Current");
        mainMotorStatorCurrent = new LiveData(elevatorMainMotor.getStatorCurrent(), "Elevator: Main Motor Stator Current");

        canCoderVelocity = new LiveData(elevatorCANcoder.getVelocity().getValueAsDouble(), "Elevator: CANCoder Velocity");
        

        // bottomSensorData = new LiveData(getBottomLimitSwitch(), "Elevator Bottom Limit Switch");
        // elevatorHeight = new LiveData(getElevatorHeight(), "Get Elevator Height"); 

        SmartDashboard.putBoolean("Elevator: Open Loop Control", false);

        globalOffset = 0.0;
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
    // public void setElevatorPositionVoltage(double position) {
    //     elevatorSetpoint.setNumber(position);
    //     elevatorMainMotor.setPositionVoltage(position);
    // }

    /**
     * Commands elevatorMainMotor to a designated position with MotionMagic voltage
     * (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setElevatorPositionMotionMagicVoltage(double position) {
        elevatorSetpoint.setNumber(position + globalOffset);
        elevatorMainMotor.setPositionMotionMagicVoltage(position + globalOffset, 0);
    }

    /**
     * Commands elevatorMainMotor to a designated position with MotionMagic
     * TorqueCurrentFOC (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    // public void setElevatorPositionMotionMagicTorqueCurrentFOC(double position) {
    //     elevatorSetpoint.setNumber(position);
    //     elevatorMainMotor.setPositionMotionMagicTorqueCurrentFOC(position);
    // }

    public void setElevatorNeutralMode(){
        elevatorMainMotor.setNeutralControl();
    }

    public void disableSoftLimits(){
        elevatorMainMotor.setSoftLimits(false, ElevatorConstants.kElevatorForwardSoftLimit, ElevatorConstants.kElevatorReverseSoftLimit);
    }

    public void enableSoftLimits(){
        elevatorMainMotor.setSoftLimits(true, ElevatorConstants.kElevatorForwardSoftLimit, ElevatorConstants.kElevatorReverseSoftLimit);
    }

    /**
     * resets encoder on main elevator motor
     */
    public void resetElevatorPosition() {
        elevatorMainMotor.resetEncoder();
    }

    public void resetElevatorCANCoder(){
        elevatorCANcoder.setPosition(0);
    }

    // Accessor methods

    /**
     * @return returns cancoder reading from elevator in rotations, more accurate than just encoder
     */
    public double getElevatorCANcoderPosition() {
        return elevatorCANcoder.getPosition().getValueAsDouble();
    }

    public double getElevatorCANcoderVelocity(){
        return elevatorCANcoder.getVelocity().getValueAsDouble();
    }

    public boolean isAtPosition(double desiredPosition) {
        double epsilon;
        if (DriverStation.isAutonomous())
            epsilon = ElevatorConstants.kElevatorPositionEpsilonAuto;
        else
            epsilon = ElevatorConstants.kElevatorPositionEpsilon;
        return Math.abs(getElevatorCANcoderPosition() - (desiredPosition + globalOffset)) < epsilon;
    }
    public boolean isAtBottom() {
        return Math.abs(getElevatorCANcoderPosition() - globalOffset) < ElevatorConstants.kElevatorNeutralModePositionEpsilon;
    }

    /**
     * @return position reading of the elevatorMainMotor encoder (motor encoder
     *         units)
     */
    public double getElevatorMotorEncoderPosition() {
        return elevatorMainMotor.getPosition();
    }

    /**
     * @return elevator main motor TorqueCurrent draw (amps)
     */
    public double getMainMotorTorqueCurrent() {
        return elevatorMainMotor.getTorqueCurrent();
    }

    /**
     * @return elevator follower motor TorqueCurrent draw (amps)
     */
    public double getFollowerMotorTorqueCurrent() {
        return elevatorFollowerMotor.getTorqueCurrent();
    }

    /**
     * @return elevator main motor stator current draw (amps)
     */
    public double getMainMotorStatorCurrent() {
        return elevatorMainMotor.getStatorCurrent();
    }

    /**
     * @return elevator main motor stator current draw (amps)
     */
    public double getFollowerMotorStatorCurrent() {
        return elevatorFollowerMotor.getStatorCurrent();
    }

    /**
     * @return elevator main motor supply current draw (amps)
     */
    public double getMainMotorSupplyCurrent() {
        return elevatorMainMotor.getSupplyCurrent();
    }

    /**
     * @return elevator follower motor supply current draw (amps)
     */
    public double getFollowerMotorSupplyCurrent() {
        return elevatorFollowerMotor.getSupplyCurrent();
    }

    /**
     * @return elevator main motor velocity (rotations per second)
     */
    public double getElevatorMainMotorVelocity(){
        return elevatorMainMotor.getRPS();
    }

    /**
     * @return elevator follower motor velocity (rotations per second)
     */
    public double getElevatorFollowerMotorVelocity(){
        return elevatorFollowerMotor.getRPS();
    }

    /**
     * @return elevator position setpoint (CANcoder value)
     */
    public double getElevatorSetpoint(){
        return elevatorSetpoint.getNumber();
    }

    public double getElevatorMainMotorTemperature() {
        return elevatorMainMotor.getMotorTemperature();
    }

    public double getElevatorFollowerMotorTemperature() {
        return elevatorFollowerMotor.getMotorTemperature();
    }

    public double getCANCoderVelocity(){
        return elevatorCANcoder.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ELEVATOR CANCODER VELOCITY", getCANCoderVelocity());


        if(SmartDashboard.getBoolean("Elevator: Open Loop Control", false)){
            setElevatorPercentOutput(DriverOI.getInstance().getRightForward() * 0.3);
        }
        SmartDashboard.putNumber("Elevator: Commanded Percent Output", DriverOI.getInstance().getForward() * 0.3);

        elevatorEncoderPosition.setNumber(getElevatorMotorEncoderPosition());
        elevatorCanCoderPosition.setNumber(getElevatorCANcoderPosition());
        mainMotorTemp.setNumber(getElevatorMainMotorTemperature());
        followerMotorTemp.setNumber(getElevatorFollowerMotorTemperature());

        mainMotorSupplyCurrent.setNumber(getMainMotorSupplyCurrent());
        mainMotorStatorCurrent.setNumber(getMainMotorStatorCurrent());

        canCoderVelocity.setNumber(getElevatorCANcoderVelocity());
        
        globalOffset = SmartDashboard.getNumber("Elevator: Global Offset", 0);
    }

    @Override
    public void simulationPeriodic() {
    }
}
