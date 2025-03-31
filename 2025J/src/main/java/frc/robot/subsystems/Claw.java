package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.LiveData;
import frc.robot.utils.RobotMap;

public class Claw extends SubsystemBase {

    private static Claw claw;
    private Kraken coralMotor, algaeMotor;
    private CANrange topSensor, bottomSensor, algaeSensor;
    private CANrangeConfiguration clawTopSensorConfig, clawBottomSensorConfig, clawAlgaeSensorConfig;

    private LiveData topSensorData, bottomSensorData, motorTemp, motorCurrent, 
        topSensorDistance, bottomSensorDistance,
        position, velocity, hasAlgae;  

    public Claw() {
        coralMotor = new Kraken(RobotMap.CLAW_CORAL_MOTOR_ID, RobotMap.RIO_BUS);
        algaeMotor = new Kraken(RobotMap.CLAW_ALGAE_MOTOR_ID, RobotMap.RIO_BUS);

        topSensor = new CANrange(RobotMap.CLAW_TOP_SENSOR_ID, RobotMap.RIO_BUS);
        clawTopSensorConfig = new CANrangeConfiguration();

        bottomSensor = new CANrange(RobotMap.CLAW_BOTTOM_SENSOR_ID, RobotMap.RIO_BUS);
        clawBottomSensorConfig = new CANrangeConfiguration();

        algaeSensor = new CANrange(RobotMap.CLAW_ALGAE_SENSOR_ID, RobotMap.RIO_BUS);
        clawAlgaeSensorConfig = new CANrangeConfiguration();

        coralMotor.setInverted(false);
        coralMotor.setSupplyCurrentLimit(ClawConstants.kClawCoralSupplyCurrentLimit);
        coralMotor.setStatorCurrentLimit(ClawConstants.kClawCoralStatorCurrentLimit);
        coralMotor.setBrake();
        coralMotor.setPIDValues(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD, ClawConstants.kFF);

        algaeMotor.setInverted(true);
        algaeMotor.setSupplyCurrentLimit(ClawConstants.kClawAlgaeSupplyCurrentLimit);
        algaeMotor.setStatorCurrentLimit(ClawConstants.kClawAlgaeStatorCurrentLimit);
        algaeMotor.setBrake();

        configureCANrange(topSensor, clawTopSensorConfig, Constants.ClawConstants.kTopSensorSignalStrength,
                Constants.ClawConstants.kTopSensorProximityThreshold,
                Constants.ClawConstants.kTopSensorProximityHysteresis,
                Constants.ClawConstants.kTopSensorFovCenterX,
                Constants.ClawConstants.kTopSensorFovCenterY,
                Constants.ClawConstants.kTopSensorFovRangeX,
                Constants.ClawConstants.kTopSensorFovRangeY);
                
        configureCANrange(bottomSensor, clawBottomSensorConfig, Constants.ClawConstants.kBottomSensorSignalStrength,
                Constants.ClawConstants.kBottomSensorProximityThreshold,
                Constants.ClawConstants.kBottomSensorProximityHysteresis,
                Constants.ClawConstants.kBottomSensorFovCenterX,
                Constants.ClawConstants.kBottomSensorFovCenterY,
                Constants.ClawConstants.kBottomSensorFovRangeX,
                Constants.ClawConstants.kBottomSensorFovRangeY);
            
        configureCANrange(algaeSensor, clawAlgaeSensorConfig, Constants.ClawConstants.kAlgaeSensorSignalStrength,
                Constants.ClawConstants.kAlgaeSensorProximityThreshold,
                Constants.ClawConstants.kAlgaeSensorProximityHysteresis,
                Constants.ClawConstants.kAlgaeSensorFovCenterX,
                Constants.ClawConstants.kAlgaeSensorFovCenterY,
                Constants.ClawConstants.kAlgaeSensorFovRangeX,
                Constants.ClawConstants.kAlgaeSensorFovRangeY);

                topSensorData = new LiveData(getTopSensor(), "Claw: Top Sensor");
                bottomSensorData = new LiveData(getBottomSensor(), "Claw: Bottom Sensor");
                
        
                topSensorDistance = new LiveData(getTopSensorDistance(), "Claw: Top Sensor Distance");
                bottomSensorDistance = new LiveData(getBottomSensorDistance(), "Claw: Bottom Sensor Distance");
        
                motorTemp = new LiveData(coralMotor.getMotorTemperature(), "Claw: Coral Motor Temp"); 
                motorCurrent = new LiveData(coralMotor.getSupplyCurrent(), "Claw: Coral Motor Supply Current");
                position = new LiveData(getCoralMotorPosition(), "Claw: position");
                velocity = new LiveData(getCoralMotorVelocity(), "Claw: Velocity RPM");
                hasAlgae = new LiveData(getAlgaeSensor(), "Claw: Has Algae");
                // hasCoral = new LiveData(hasCoral(), "Claw Has Coral");
                // coralIndexed = new LiveData(coralIndexed(), "Claw Coral is Indexed");

        SmartDashboard.putNumber("Claw Increment", 5);
    }

    public void configureCANrange(CANrange sensor, CANrangeConfiguration config, double signalStrengthThreshold,
        double proximityThreshold, double proximityHysteresis) {
        config.ProximityParams.ProximityThreshold = proximityThreshold;
        config.ProximityParams.MinSignalStrengthForValidMeasurement = signalStrengthThreshold;
        config.ProximityParams.ProximityHysteresis = proximityHysteresis;
        sensor.getConfigurator().apply(config);
    }

    public void configureCANrange(CANrange sensor, CANrangeConfiguration config, double signalStrengthThreshold,
        double proximityThreshold, double proximityHysteresis, double fovCenterX, double fovCenterY, double fovRangeX, double fovRangeY) {
        config.ProximityParams.ProximityThreshold = proximityThreshold;
        config.ProximityParams.MinSignalStrengthForValidMeasurement = signalStrengthThreshold;
        config.ProximityParams.ProximityHysteresis = proximityHysteresis;

        config.FovParams.FOVCenterX = fovCenterX;
        config.FovParams.FOVCenterY = fovCenterY;
        config.FovParams.FOVRangeX = fovRangeX;
        config.FovParams.FOVRangeY = fovRangeY;
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
    public void setCoralMotor(double speed) {
        coralMotor.setPercentOutput(speed);
    }


    public void setAlgaeMotor(double speed) {
        algaeMotor.setPercentOutput(speed);
    }

    public void stopClaw() {
        setCoralMotor(0);
        setAlgaeMotor(0);
    }

    public void stopCoralMotor(){
        setCoralMotor(0);
    }

    public void stopAlgaeMotor(){
        setAlgaeMotor(0);
    }

    /**
     * Sets clawMotor speed to the designated percent output listed in the
     * ClawConstants class
     */
    public void outtakeCoral(){
        setCoralMotor(ClawConstants.kCoralOuttakeSpeed);
    }

    public void outtakeCoralL1(){
        setCoralMotor(ClawConstants.kCoralL1OuttakeSpeed);
    }

    // speed argument used for fast and slow coral intake speeds
    public void intakeCoral(double speed){
        setCoralMotor(speed);
    }

    public void outtakeAlgae(){
        setAlgaeMotor(ClawConstants.kAlgaeOuttakeSpeed);
    }

    public void intakeAlgae(){
        setAlgaeMotor(ClawConstants.kAlgaeIntakeSpeed);
    }

    public void holdAlgae() {
        setAlgaeMotor(ClawConstants.kAlgaeHoldSpeed);
    }

    public void incrementClaw() {
        coralMotor.setPositionVoltage(coralMotor.getPosition() + ClawConstants.kCoralPositionIncrement);
    }

    // Accessor methods

    /**
     * @return claw coral algae sensor reading (digital sensor) as boolean
     */
    public boolean getTopSensor() {
        // return topSensor.getIsDetected().getValue(); // true if detected
        // return topSensor.getDistance().getValueAsDouble() < 0.16;
        return topSensor.getAmbientSignal().getValueAsDouble() < 5;
    }

    public boolean getBottomSensor() {
        // return bottomSensor.getIsDetected().getValue(); // true if detected
        // return bottomSensor.getDistance().getValueAsDouble() < 0.1;
        return bottomSensor.getAmbientSignal().getValueAsDouble() < 10;
    }

    public boolean getAlgaeSensor() {
        return algaeSensor.getAmbientSignal().getValueAsDouble() < 30.0 && algaeSensor.getDistance().getValueAsDouble() < 0.05; // true if detected
    }
    
    public boolean eitherCoralSensorsTriggeredAndNoAlgae() {
        return Claw.getInstance().eitherCoralSensorTriggered() && !Claw.getInstance().getAlgaeSensor();
    }

    /**
     * @return claw algae coral distance sensor reading (distance sensor units)
     */
    public double getTopSensorDistance() {
        return topSensor.getDistance().getValueAsDouble();
    }

    public double getBottomSensorDistance() {
        return bottomSensor.getDistance().getValueAsDouble();
    }

    public double getAlgaeSensorDistance() {
        return algaeSensor.getDistance().getValueAsDouble();
    }

    /**
     * @return claw motor stator current draw (amps)
     */
    public double getCoralMotorStatorCurrent() {
        return coralMotor.getStatorCurrent();
    }

    public double getAlgaeMotorStatorCurrent(){
        return algaeMotor.getStatorCurrent();
    }

    /**
     * @return claw motor supply current draw (amps)
     */
    public double getCoralMotorSupplyCurrent() {
        return coralMotor.getSupplyCurrent();
    }

    public double getAlgaeMotorSupplyCurrent(){
        return algaeMotor.getSupplyCurrent();
    }

    /**
     * @return position of clawMotor encoder (mechanism rotations)
     */
    public double getCoralMotorPosition() {
        return coralMotor.getPosition();
    }

    public double getAlgaeMotorPosition(){
        return algaeMotor.getPosition();
    }

    /**
     * @return velocity of clawMotor encoder (rotor rotations per minute)
     */
    public double getCoralMotorVelocity() {
        return coralMotor.getRPM();
    }

    public double getAlgaeMotorVelocity(){
        return algaeMotor.getRPM();
    }

    /**
     * @return returns if either sensor has a coral
     */
    public boolean eitherCoralSensorTriggered() {
        return getTopSensor() || getBottomSensor();
    }

    /**
     * @return returns if ready to shoot, sensor 2 detects the coral but not sensor
     *         1
     */
    public boolean bothCoralSensorsTriggered() {
        return getBottomSensor() && getTopSensor();
    }

    public double getCoralMotorTemperature() {
        return coralMotor.getMotorTemperature();
    }

    public double getAlgaeMotorTemperature() {
        return algaeMotor.getMotorTemperature();
    }

    @Override
    public void periodic() {
        topSensorData.setBoolean(getTopSensor());
        bottomSensorData.setBoolean(getBottomSensor()); 
        topSensorDistance.setNumber(getTopSensorDistance());
        bottomSensorDistance.setNumber(getBottomSensorDistance());
        motorTemp.setNumber(getCoralMotorTemperature()); 
        motorCurrent.setNumber(getCoralMotorSupplyCurrent());
        position.setNumber(getCoralMotorPosition()); 
        velocity.setNumber(getCoralMotorVelocity());
        hasAlgae.setBoolean(getAlgaeSensor()); 
        SmartDashboard.putNumber("Algae Sensor Distance", getAlgaeSensorDistance());
    }

    @Override
    public void simulationPeriodic() {

    }
}
