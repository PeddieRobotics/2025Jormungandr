package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClimberConstants;
import frc.robot.utils.DriverOI;
import frc.robot.utils.Kraken;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.RobotMap;



public class Climber extends SubsystemBase {
    private static Climber instance;
    private Kraken climberMotor;


    public Climber() {

        climberMotor = new Kraken(RobotMap.CLIMBER_MAIN_MOTOR_ID, RobotMap.CANIVORE_BUS);
        climberMotor.setEncoder(0.0);

        climberMotor.setSupplyCurrentLimit(40.0);
        climberMotor.setStatorCurrentLimit(40.0);

        climberMotor.setSoftLimits(true, 300, 0);

        climberMotor.setInverted(true);
        climberMotor.setBrake();

        SmartDashboard.putBoolean("Climber: Open Loop Control", false);
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public void retractClimber(){
        double speed = -0.75;
        if(climbRetracted()){
           setSpeed(0.0); 
        }else{
            setSpeed(speed);
        }
    }

    public void deployClimber(){
        if(climbDeployed()){
            setSpeed(0.0);
        }else{
            setSpeed(ClimberConstants.kClimberDeployedPercentOutput);
        }
    }

    public boolean climbRetracted(){
        return getClimberPosition() < ClimberConstants.kClimberRetractedPosition;

    }

    public boolean climbDeployed(){
        return getClimberPosition() > ClimberConstants.kClimberDeployedPosition;
    }

    /**
     * Sets leftClimberMotor spe
     * ed to a designated percent output (open loop control)
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
    public double getClimberStatorCurrent() {
        return climberMotor.getStatorCurrent();
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
        SmartDashboard.putNumber("Climber: commanded output", OperatorOI.getInstance().getForward());
        SmartDashboard.putBoolean("Climber: deployed", climbDeployed());
        SmartDashboard.putBoolean("Climber: retracted", climbRetracted());
        SmartDashboard.putNumber("Climber: position", getClimberPosition());

    }
}
