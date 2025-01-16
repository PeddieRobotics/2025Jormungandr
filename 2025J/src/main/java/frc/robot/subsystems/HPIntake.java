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
    
    public void setIntake(double speed){
        rollerMotor.setMotor(speed);
    }

    public void stopIntake(){
        setIntake(0.0);
    }

    public void runIntake(){
        setIntake(IntakeConstants.kHPIntakeSpeed);
    }

    public void reverseIntake(){
        setIntake(-IntakeConstants.kHPIntakeSpeed);
    }
    
}
