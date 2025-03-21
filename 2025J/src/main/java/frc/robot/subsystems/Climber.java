package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClimberConstants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Kraken;
import frc.robot.utils.LiveData;
import frc.robot.utils.RobotMap;



public class Climber extends SubsystemBase {
    private static Climber instance;
    private Kraken climberMotor;


    public Climber() {

        climberMotor = new Kraken(RobotMap.CLIMBER_MAIN_MOTOR_ID, RobotMap.CANIVORE_BUS);
        climberMotor.setEncoder(0.0);

        climberMotor.setInverted(false);
        climberMotor.setBrake();

        SmartDashboard.putBoolean("Climber: Open Loop Control", false);
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    /**
     * Sets leftClimberMotor speed to a designated percent output (open loop control)
     * ex: input of 0.5 will run 50% of its max speed forward
     * 
     * @param speed - Percent of leftClimberMotor's speed [-1.0, 1.0]
     */
    public void setSpeed(double speed) {
        climberMotor.setPercentOutput(speed);
    }

    /**
     * @return returns climber motor supply current draw (amps)
     */
    public double getClimberSupplyCurrent() {
        return climberMotor.getSupplyCurrent();
    }

    /**
     * @return returns temperature of climber motor in celcius
     */
    public double getClimberTemperature() {
        return climberMotor.getMotorTemperature();
    }

    /**
     * @return returns position reading of leftClimberMotor encoder (mechanism rotations)
     */
    public double getClimberPosition() {
        return climberMotor.getPosition();
    }

    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("Climber: Open Loop Control", false)){
            climberMotor.setPercentOutput(DriverOI.getInstance().getRightForward());
        }

    }
}
