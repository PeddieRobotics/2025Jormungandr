package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LiveData;
import frc.robot.utils.Logger;
import frc.robot.utils.RobotMap;

public class HPIntake extends SubsystemBase{

    private static HPIntake hpIntake;
    private final Servo linearActuatorRight, linearActuatorLeft;

    @SuppressWarnings("unused")
    private LiveData rollerMotorStatorCurrent, rollerMotorTemperature, pivotMotorStatorCurrent, pivotMotorTemperature, 
    intakePosition, intakeVelocity, pivotMotorSupplyCurrent, rollerMotorSupplyCurrent; 
    
    public HPIntake() {
        linearActuatorRight = new Servo(RobotMap.HP_INTAKE_SERVO_LEFT_ID);
        linearActuatorLeft = new Servo(RobotMap.HP_INTAKE_SERVO_RIGHT_ID);
        linearActuatorRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        linearActuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

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
        Logger.getInstance().logEvent("Extend Linear Actuator", true);
        linearActuatorRight.set(1);
        linearActuatorLeft.set(1);
    }

    public void retractLinearActuator(){
        Logger.getInstance().logEvent("Retract Linear Actuator", true);
        linearActuatorRight.set(0);
        linearActuatorLeft.set(0);
    }

    public void toggleLinearActuator() {
        Logger.getInstance().logEvent("Toggle Linear Actuator", true);
        if (linearActuatorLeft.get() == 1)
            retractLinearActuator();
        else 
            extendLinearActuator();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
