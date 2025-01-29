package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class Claw extends SubsystemBase {

    private static Claw claw;
    private Kraken clawMotor;

    private DigitalInput coralSensor;
    private AnalogInput algaeSensor;

    public Claw() {
        clawMotor = new Kraken(RobotMap.CLAW_MOTOR_ID, RobotMap.CANIVORE_NAME);

        coralSensor = new DigitalInput(RobotMap.CLAW_CORAL_SENSOR_ID);
        algaeSensor = new AnalogInput(RobotMap.CLAW_ALGAE_SENSOR_ID);

        clawMotor.setInverted(false);
        clawMotor.setStatorCurrentLimit(ClawConstants.kClawStatorCurrentLimit);
        clawMotor.setBrake();
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
    public void intakePiece() {
        setClaw(ClawConstants.kClawIntakeSpeed);
    }

    /**
     * Sets clawMotor speed to the designated percent output listed in the
     * ClawConstants class
     */
    public void outtakePiece() {
        setClaw(ClawConstants.kClawOuttakeSpeed);
    }

    // Accessor methods

    /**
     * @return claw coral sensor reading (digital sensor)
     */
    public boolean getCoralSensor() {
        return coralSensor.get();
    }

    /**
     * @return claw algae distance sensor reading (distance sensor units)
     */
    public double getAlgaeSensor() {
        return algaeSensor.getVoltage();
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

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

}
