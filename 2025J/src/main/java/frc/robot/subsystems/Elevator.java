package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ElevatorConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class Elevator extends SubsystemBase {
    private static Elevator elevator;
    private final Kraken elevatorMainMotor, elevatorFollowerMotor;

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

        elevatorMainMotor.setSoftLimits(true, ElevatorConstants.kElevatorForwardSoftLimit, ElevatorConstants.kElevatorReverseSoftLimit);
        elevatorFollowerMotor.setSoftLimits(true, ElevatorConstants.kElevatorForwardSoftLimit, ElevatorConstants.kElevatorReverseSoftLimit);

        // TODO: setup cancoder

    }

    public Elevator getInstance(){
        if(elevator == null){
            elevator = new Elevator();
        }
        return elevator;
    }

    public void setElevatorPercentOutput(double speed){
        elevatorMainMotor.setMotor(speed);
    }

    public void stopElevator(){
        elevatorMainMotor.setMotor(0);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
