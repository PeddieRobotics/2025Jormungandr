package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.IntakeConstants;

public class HPIntake extends SubsystemBase{

    private static HPIntake hpIntake;
    private final Kraken rollerMotor;
    

    public HPIntake() {
        rollerMotor = new Kraken(RobotMap.HP_INTAKE_ID, RobotMap.CANIVORE_NAME);

        // TODO: update to reflect desired behavior
        rollerMotor.setInverted(false);
        rollerMotor.setSupplyCurrentLimit(IntakeConstants.kHPIntakeMotorCurrentLimit);
        rollerMotor.setBrake();
    }

    public static HPIntake getInstance(){
        if(hpIntake == null){
            hpIntake = new HPIntake();
        }
        return hpIntake;
    }
    
    /**
     * Sets rollerMotor speed to a designated percent output
     * 
     * @param speed Percent of rollerMotor's speed
     */
    public void setIntake(double speed){
        rollerMotor.setMotor(speed);
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
    
}
