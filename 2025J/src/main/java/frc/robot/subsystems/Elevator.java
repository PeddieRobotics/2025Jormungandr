package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.ElevatorConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class Elevator extends SubsystemBase {
    private static Elevator elevator;
    private Kraken elevatorMainMotor, elevatorFollowerMotor;
    private DigitalInput bottomLimitSwitch;

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

        elevatorMainMotor.setVelocityPIDValues(ElevatorConstants.kS, ElevatorConstants.kV,
                ElevatorConstants.kA,
                ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
                ElevatorConstants.kFF);
        elevatorFollowerMotor.setVelocityPIDValues(ElevatorConstants.kS, ElevatorConstants.kV,
                ElevatorConstants.kA,
                ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
                ElevatorConstants.kFF);

        elevatorMainMotor.setMotionMagicParameters(ElevatorConstants.kElevatorMaxCruiseVelocity, ElevatorConstants.kElevatorMaxCruiseAcceleration, ElevatorConstants.kElevatorMaxCruiseJerk);
        elevatorFollowerMotor.setMotionMagicParameters(ElevatorConstants.kElevatorMaxCruiseVelocity, ElevatorConstants.kElevatorMaxCruiseAcceleration, ElevatorConstants.kElevatorMaxCruiseJerk);

        elevatorMainMotor.setSoftLimits(true, ElevatorConstants.kElevatorForwardSoftLimit, ElevatorConstants.kElevatorReverseSoftLimit);
        elevatorFollowerMotor.setSoftLimits(true, ElevatorConstants.kElevatorForwardSoftLimit, ElevatorConstants.kElevatorReverseSoftLimit);

        bottomLimitSwitch = new DigitalInput(RobotMap.ELEVATOR_LIMIT_SWITCH_ID);
    }

    /**
     * @return the existing elevator instance or creates it if it doesn't exist
     */
    public static Elevator getInstance(){
        if(elevator == null){
            elevator = new Elevator();
        }
        return elevator;
    }

    /**
     * Sets elevatorMainMotor speed to a designated percent output (open loop control)
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
     * Commands elevatorMainMotor to a designated position with position voltage PID (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setElevatorPositionVoltage(double position){
        elevatorMainMotor.setPositionVoltage(position);
    }

    /**
     * Commands elevatorMainMotor to a designated position with MotionMagic voltage (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setElevatorPositionMotionMagicVoltage(double position){
        elevatorMainMotor.setPositionMotionMagicVoltage(position);
    }

    /**
     * Commands elevatorMainMotor to a designated position with MotionMagic TorqueCurrentFOC (closed loop control)
     * 
     * @param position - commanded motor position (motor encoder units)
     */
    public void setElevatorPositionMotionMagicTorqueCurrentFOC(double position){
        elevatorMainMotor.setPositionMotionMagicTorqueCurrentFOC(position);
    }

    //Accessor methods

    /**
     * @return whether elevator bottom limit switch is triggered
     */
    public boolean getBottomLimitSwitch(){
        return bottomLimitSwitch.get();
    }

    /**
     * @return position reading of the elevatorMainMotor encoder (motor encoder units)
     */
    public double getElevatorPosition(){
        return elevatorMainMotor.getPosition();
    }

    /**
     * @return elevator motor TorqueCurrent draw (amps)
     */
    public double getMotorTorqueCurrent(){
        return elevatorMainMotor.getTorqueCurrent();
    }

    /**
     * @return elevator motor stator current draw (amps)
     */
    public double getMotorStatorCurrent(){
        return elevatorMainMotor.getStatorCurrent();
    }
    
    /**
     * @return elevator motor supply current draw (amps)
     */
    public double getMotorSupplyCurrent(){
        return elevatorMainMotor.getSupplyCurrent();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
