package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private DigitalInput bottomLimitSwitch;
    private TunableConstant kP, kS, kV, kI, kD, kFF, kA,
        kElevatorMaxCruiseVelocity, kElevatorMaxCruiseAcceleration, kElevatorMaxCruiseJerk, kElevatorForwardSoftLimit, kElevatorReverseSoftLimit,
        L1Setpoint, L2Setpoint, L3Setpoint, L4Setpoint, HPIntakeSetpoint, stowSetpoint, bargeSetpoint, algaeL1Setpoint, algaeL2Setpoint, processorSetpoint;
    
    private LiveData elevatorPosition, elevatorSetpoint, mainMotorTemp, followerMotorTemp, mainMotorCurrent, followerMotorCurrent;

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

        kP = new TunableConstant(ElevatorConstants.kP, "Elevator kP");
        kI = new TunableConstant(ElevatorConstants.kI, "Elevator kI");
        kD = new TunableConstant(ElevatorConstants.kD, "Elevator kD");
        kA = new TunableConstant(ElevatorConstants.kA, "Elevator A"); 
        kS = new TunableConstant(ElevatorConstants.kS, "Elevator kS");
        kV = new TunableConstant(ElevatorConstants.kV, "Elevator kV");
        kFF = new TunableConstant(ElevatorConstants.kFF, "Elevator kFF");
        kElevatorMaxCruiseVelocity = new TunableConstant(ElevatorConstants.kElevatorMaxCruiseVelocity, "Elevator kElevatorMaxCruiseVelocity");
        kElevatorReverseSoftLimit = new TunableConstant(ElevatorConstants.kElevatorReverseSoftLimit, "Elevator kElevatorReverseSoftLimit");
        kElevatorForwardSoftLimit = new TunableConstant(ElevatorConstants.kElevatorForwardSoftLimit, "Elevator kElevatorForwardSoftLimit");
        kElevatorMaxCruiseJerk = new TunableConstant(ElevatorConstants.kElevatorMaxCruiseJerk, "Elevator kElevatorMaxCruiseJerk");
        kElevatorMaxCruiseAcceleration = new TunableConstant(ElevatorConstants.kElevatorMaxCruiseAcceleration, "Elevator kElevatorMaxCruiseAcceleration"); 
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
        elevatorPosition = new LiveData(elevatorMainMotor.getPosition(), "Elevator Current Position"); 

        mainMotorTemp = new LiveData(elevatorMainMotor.getMotorTemperature(), "Elevator Main Motor Temp"); 
        followerMotorTemp = new LiveData(elevatorFollowerMotor.getMotorTemperature(), "Elevator Follower Motor Temp"); 
        
        mainMotorCurrent = new LiveData(elevatorMainMotor.getSupplyCurrent(), "Elevator Main Motor Current"); 
        followerMotorCurrent = new LiveData(elevatorFollowerMotor.getSupplyCurrent(), "Elevator Follower Motor Current"); 
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

    public double getElevatorHeight() {
        //TODO: implement this code
        return 0;
    }

    public boolean isAtHeight(double targetHeight) {
        return Math.abs(getElevatorHeight() - targetHeight) < ElevatorConstants.kElevatorHeightEpsilon;
    }
    @Override
    public void periodic() {
        elevatorMainMotor.setVelocityPIDValues(kS.get(), kV.get(),
            kA.get(),
            kP.get(), kI.get(), kD.get(),
            kFF.get());
        elevatorFollowerMotor.setVelocityPIDValues(kS.get(), kV.get(),
            kA.get(),
            kP.get(), kI.get(), kD.get(),
            kFF.get());

        elevatorMainMotor.setMotionMagicParameters(kElevatorMaxCruiseVelocity.get(), kElevatorMaxCruiseAcceleration.get(), kElevatorMaxCruiseJerk.get());
        elevatorFollowerMotor.setMotionMagicParameters(kElevatorMaxCruiseVelocity.get(), kElevatorMaxCruiseAcceleration.get(), kElevatorMaxCruiseJerk.get());

        elevatorMainMotor.setSoftLimits(true, kElevatorForwardSoftLimit.get(), kElevatorReverseSoftLimit.get());    
        elevatorFollowerMotor.setSoftLimits(true, kElevatorForwardSoftLimit.get(), kElevatorReverseSoftLimit.get());

        elevatorPosition.set(elevatorMainMotor.getPosition()); 
        mainMotorTemp.set(elevatorMainMotor.getMotorTemperature());
        followerMotorTemp.set(elevatorFollowerMotor.getMotorTemperature()); 

        mainMotorCurrent.set(elevatorMainMotor.getSupplyCurrent()); 
        followerMotorCurrent.set(elevatorFollowerMotor.getSupplyCurrent()); 
    }

    @Override
    public void simulationPeriodic() {
    }
}
