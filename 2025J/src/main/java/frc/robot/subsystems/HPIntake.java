package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LiveData;
import frc.robot.utils.RobotMap;

public class HPIntake extends SubsystemBase{

    private static HPIntake hpIntake;
    private final Servo linearActuator;

    private LiveData rollerMotorStatorCurrent, rollerMotorTemperature, pivotMotorStatorCurrent, pivotMotorTemperature, 
    intakePosition, intakeVelocity, pivotMotorSupplyCurrent, rollerMotorSupplyCurrent; 
    
    public HPIntake() {
        linearActuator = new Servo(RobotMap.HP_INTAKE_SERVO_ID);
        linearActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
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
    
    public void extendLinearActuator(){
        linearActuator.set(1);
    }

    public void retractLinearActuator(){
        linearActuator.set(0);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
