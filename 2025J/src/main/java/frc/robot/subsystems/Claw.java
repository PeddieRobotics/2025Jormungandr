package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class Claw extends SubsystemBase {

    private static Claw claw;
    private Kraken clawMotor;
    private CANrange coralSensor1;
    private CANrange coralSensor2;
    private CANrange algaeSensor;
    private CANrangeConfiguration coralSensor1Config;
    private CANrangeConfiguration coralSensor2Config;
    private CANrangeConfiguration algaeSensorConfig;

    public Claw() {
        clawMotor = new Kraken(RobotMap.CLAW_MOTOR_ID, RobotMap.CANIVORE_NAME);

        coralSensor1 = new CANrange(RobotMap.CLAW_CORAL_SENSOR1_ID, RobotMap.CANIVORE_NAME);
        coralSensor1Config = new CANrangeConfiguration();

        coralSensor2 = new CANrange(RobotMap.CLAW_CORAL_SENSOR2_ID, RobotMap.CANIVORE_NAME);
        coralSensor2Config = new CANrangeConfiguration();

        algaeSensor = new CANrange(RobotMap.CLAW_ALGAE_SENSOR_ID, RobotMap.CANIVORE_NAME);
        algaeSensorConfig = new CANrangeConfiguration();

        clawMotor.setInverted(false);
        clawMotor.setStatorCurrentLimit(ClawConstants.kClawStatorCurrentLimit);
        clawMotor.setBrake();

        configureCANrange(coralSensor1, coralSensor1Config, Constants.ClawConstants.kCoralSensor1SignalStrength, 
                            Constants.ClawConstants.kCoralSensor1ProximityThreshold, Constants.ClawConstants.kCoralSensor1ProximityHysteresis);
        configureCANrange(coralSensor2, coralSensor2Config, Constants.ClawConstants.kCoralSensor2SignalStrength, 
                            Constants.ClawConstants.kCoralSensor2ProximityThreshold, Constants.ClawConstants.kCoralSensor2ProximityHysteresis);
        configureCANrange(algaeSensor, algaeSensorConfig, Constants.ClawConstants.kAlgaeSensorSignalStrength, 
                            Constants.ClawConstants.kAlgaeSensorProximityThreshold, Constants.ClawConstants.kAlgaeSensorProximityHysteresis);
    }

    public void configureCANrange(CANrange sensor, CANrangeConfiguration config, double signalStrengthThreshold, double proximityThreshold, double proximityHysteresis){
        config.ProximityParams.ProximityThreshold = proximityThreshold;
        config.ProximityParams.MinSignalStrengthForValidMeasurement = signalStrengthThreshold;
        config.ProximityParams.ProximityHysteresis = proximityHysteresis;
        sensor.getConfigurator().apply(config);
    }

    /**
     * @return the existing claw instance or creates it if it doesn't exist
     */
    public static Claw getInstance() {
        if (claw == null) {
            claw = new Claw();
        }
        return claw;
    }

    /**
     * Sets clawMotor speed to a designated percent output (open loop control)
     * 
     * @param speed - Percent of clawMotor's speed (-1.0 to 1.0)
     */
    public void setClaw(double speed) {
        clawMotor.setPercentOutput(speed);
    }

    public void stopClaw(){
        setClaw(0);
    }

    /**
     * Sets clawMotor speed to the designated percent output listed in the
     * ClawConstants class
     */
    public void intakePiece(double speed) {
        setClaw(speed);
    }

    /**
     * Sets clawMotor speed to the designated percent output listed in the
     * ClawConstants class
     */
    public void outtakePiece() {
        setClaw(ClawConstants.kCoralOuttakeSpeed);
    }

    public void holdAlgae(){
        setClaw(ClawConstants.kAlgaeHoldSpeed);
    }
    // Accessor methods

    /**
     * @return claw coral algae sensor reading (digital sensor) as boolean
     */
    public boolean getCoralSensor1() {
        return coralSensor1.getIsDetected().getValue();  //true if detected
    }

    public boolean getCoralSensor2() {
        return coralSensor2.getIsDetected().getValue();  //true if detected
    }

    public boolean getAlgaeSensor() {
        return algaeSensor.getIsDetected().getValue();  //true if detected
    }

    /**
     * @return claw algae coral distance sensor reading (distance sensor units)
     */
    public double getCoralSensor1Distance() {
        return coralSensor1.getDistance().getValueAsDouble();
    }

    public double getCoralSensor2Distance() {
        return coralSensor2.getDistance().getValueAsDouble();
    }
    
    public double getAlgaeSensorDistance() {
        return algaeSensor.getDistance().getValueAsDouble();
    }

    /**
     * @return claw motor stator current draw (amps)
     */
    public double getMotorStatorCurrent() {
        return clawMotor.getStatorCurrent();
    }

    /**
     * @return claw motor supply current draw (amps)
     */
    public double getMotorSupplyCurrent() {
        return clawMotor.getSupplyCurrent();
    }

    public double getPosition(){
        return clawMotor.getPosition();
    }

    public double getVelocity(){
        return clawMotor.getRPM();
    }

    public boolean hasCoral(){
        return getCoralSensor1() || getCoralSensor2();
    }

    public boolean coralIndexed(){
        return getCoralSensor2() && !getCoralSensor1();
    }

    public boolean hasAlgae(){
        return getAlgaeSensor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Coral Sensor 1", getCoralSensor1());
        SmartDashboard.putBoolean("Coral Sensor 2", getCoralSensor2());
        SmartDashboard.putNumber("Algae Sensor", getAlgaeSensorDistance());
    }

    @Override
    public void simulationPeriodic() {

    }
}
