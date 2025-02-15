package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class HPIntake extends SubsystemBase{

    private static HPIntake hpIntake;
    private final Kraken rollerMotor;
    private final Kraken pivotMotor;
    
    public HPIntake() {
        rollerMotor = new Kraken(RobotMap.HP_INTAKE_ROLLER_ID, RobotMap.CANIVORE_NAME);
        pivotMotor = new Kraken(RobotMap.HP_INTAKE_PIVOT_ID, RobotMap.CANIVORE_NAME);

        // TODO: update to reflect desired behavior
        rollerMotor.setInverted(false);
        rollerMotor.setSupplyCurrentLimit(IntakeConstants.kHPIntakeRollerSupplyCurrentLimit);
        rollerMotor.setBrake();

        pivotMotor.setInverted(false);
        pivotMotor.setSupplyCurrentLimit(IntakeConstants.kHPIntakePivotSupplyCurrentLimit);
        pivotMotor.setBrake();
        pivotMotor.setPIDValues(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA, IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, IntakeConstants.kFF);
    }

    /**
     * @return the existing HPIntake instance or creates it if it doesn't exist
     */
    public static HPIntake getInstance(){
        if(hpIntake == null){
            hpIntake = new HPIntake();
        }
        return hpIntake;
    }
    
    /**
     * Sets rollerMotor speed to a designated percent output (open loop control)
     * 
     * @param speed - Percent of rollerMotor's speed (-1.0 to 1.0)
     */
    public void setIntake(double speed){
        rollerMotor.setPercentOutput(speed);
    }

    /**
     * Stops rollerMotor by setting the speed to 0
     */
    public void stopIntake(){
        setIntake(0);
    }

    /**
     * Sets rollerMotor speed to the designated percent output listed in the IntakeConstants class
     */
    public void runIntake(){
        setIntake(IntakeConstants.kHPIntakeSpeed);
    }

    /**
     * Sets rollerMotor speed to the opposite of the designated percent output listed in the IntakeConstants class
     */
    public void reverseIntake(){
        setIntake(-1*IntakeConstants.kHPIntakeSpeed);
    }

    /**
     * Sets pivotMotor position to the specific position
     */
    public void setIntakePosition(double position){
        pivotMotor.setPositionVoltage(position);
    }

    public double getIntakePosition(){
        return pivotMotor.getPosition();
    }

    public double getIntakeVelocity(){
        return pivotMotor.getRPS();
    }

    //Accessor methods

    /**
     * @return HP intake motor stator current draw (amps)
     */
    public double getMotorStatorCurrent(){
        return rollerMotor.getStatorCurrent();
    }

    /**
     * @return claw motor supply current draw (amps)
     */
    public double getMotorSupplyCurrent(){
        return rollerMotor.getSupplyCurrent();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
