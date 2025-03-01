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

        // rollerMotorStatorCurrent = new LiveData(getRollerMotorStatorCurrent(), "HP Intake Roller Motor Stator Current");
        // rollerMotorTemperature = new LiveData(rollerMotor.getMotorTemperature(), "HP Intake Roller Motor Temp");
        // pivotMotorStatorCurrent = new LiveData(getPivotMotorStatorCurrent(), "HP Intake Pivot Motor Stator Current");
        // pivotMotorTemperature = new LiveData(pivotMotor.getMotorTemperature(), "HP Pivot Motor Temp");
        // intakePosition = new LiveData(getIntakePosition(), "HP Intake Position");
        // intakeVelocity = new LiveData(getIntakeVelocity(), "HP Intake Velocity");
        // rollerMotorSupplyCurrent = new LiveData(getRollerMotorSupplyCurrent(), "HP Intake Roller Supply Current"); 
        // pivotMotorSupplyCurrent = new LiveData(getPivotMotorSupplyCurrent(), "HP Intake Pivot Motor Supply Current"); 
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
