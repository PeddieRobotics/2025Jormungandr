package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class Claw extends SubsystemBase {

    private static Claw claw;
    private Kraken clawMotor;
    private CANrange topSensor;
    private CANrange bottomSensor;
    private CANrangeConfiguration clawTopSensorConfig;
    private CANrangeConfiguration clawBottomSensorConfig;

    public Claw() {
        clawMotor = new Kraken(RobotMap.CLAW_MOTOR_ID, RobotMap.CANIVORE_NAME);

        topSensor = new CANrange(RobotMap.CLAW_TOP_SENSOR_ID, RobotMap.CANIVORE_NAME);
        clawTopSensorConfig = new CANrangeConfiguration();

        bottomSensor = new CANrange(RobotMap.CLAW_BOTTOM_SENSOR_ID, RobotMap.CANIVORE_NAME);
        clawBottomSensorConfig = new CANrangeConfiguration();

        clawMotor.setInverted(false);
        clawMotor.setSupplyCurrentLimit(ClawConstants.kClawSupplyCurrentLimit);
        clawMotor.setStatorCurrentLimit(ClawConstants.kClawStatorCurrentLimit);
        clawMotor.setBrake();
        clawMotor.setPIDValues(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD, ClawConstants.kFF);

        configureCANrange(topSensor, clawTopSensorConfig, Constants.ClawConstants.kTopSensorSignalStrength,
                Constants.ClawConstants.kTopSensorProximityThreshold,
                Constants.ClawConstants.kTopSensorProximityHysteresis);
        configureCANrange(bottomSensor, clawBottomSensorConfig, Constants.ClawConstants.kBottomSensorSignalStrength,
                Constants.ClawConstants.kBottomSensorProximityThreshold,
                Constants.ClawConstants.kBottomSensorProximityHysteresis);

        // TODO: DELETE THIS OR ELSE
        SmartDashboard.putNumber("Claw Increment", 5);
    }

    public void configureCANrange(CANrange sensor, CANrangeConfiguration config, double signalStrengthThreshold,
            double proximityThreshold, double proximityHysteresis) {
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

    public void stopClaw() {
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

    public void holdAlgae() {
        setClaw(ClawConstants.kAlgaeHoldSpeed);
    }

    public void incrementClaw() {
        clawMotor.setPositionVoltage(clawMotor.getPosition() + ClawConstants.kCoralPositionIncrement);
    }

    // Accessor methods

    /**
     * @return claw coral algae sensor reading (digital sensor) as boolean
     */
    public boolean getTopSensor() {
        return topSensor.getIsDetected().getValue(); // true if detected
    }

    public boolean getBottomSensor() {
        return bottomSensor.getIsDetected().getValue(); // true if detected
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

    /**
     * @return position of clawMotor encoder (mechanism rotations)
     */
    public double getPosition() {
        return clawMotor.getPosition();
    }

    /**
     * @return velocity of clawMotor encoder (rotor rotations per minute)
     */
    public double getVelocity() {
        return clawMotor.getRPM();
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

    /**
     * @return returns if sensor detects algae
     */
    // TODO: Reimplement
    public boolean hasAlgae() {
        return false;
    }

    public double getClawMotorTemperature() {
        return clawMotor.getMotorTemperature();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Claw Top Sensor", getTopSensor());
        SmartDashboard.putBoolean("Claw Bottom Sensor", getBottomSensor());

        SmartDashboard.putNumber("Claw Top Sensor Distance", getTopSensorDistance());
        SmartDashboard.putNumber("Claw Bottom Sensor Distance", getBottomSensorDistance());

        SmartDashboard.putBoolean("Claw Either Sensor Triggered", eitherCoralSensorTriggered());
    }

    @Override
    public void simulationPeriodic() {

    }
}
