package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        elevatorMainMotor.setSupplyCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit);
        elevatorFollowerMotor.setSupplyCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit);

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

    public static Elevator getInstance(){
        if(elevator == null){
            elevator = new Elevator();
        }
        return elevator;
    }

    /**
     * Sets elevatorMainMotor speed to a designated percent output
     * 
     * @param speed Percent of elevatorMainMotor's speed
     */
    public void setElevatorPercentOutput(double speed) {
        elevatorMainMotor.setMotor(speed);
    }

    /**
     * Stops elevatorMainMotor by setting the speed to 0
     */
    public void stopElevator() {
        elevatorMainMotor.setMotor(0);
    }

    public void setElevatorPositionVoltage(double position){
        elevatorMainMotor.setPosition(position);
    }

    public void setElevatorPositionMotionMagic(double position){
        elevatorMainMotor.setPositionMotionMagic(position);
    }

    public boolean getBottomLimitSwitch(){
        return bottomLimitSwitch.get();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
