package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClimberConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class Climber extends SubsystemBase {
    private static Climber instance;
    private Kraken leftClimberMotor, rightClimberMotor;

    public Climber() {
        leftClimberMotor = new Kraken(RobotMap.LEFT_CLIMBER_MOTOR_ID, RobotMap.CANIVORE_NAME);
        rightClimberMotor = new Kraken(RobotMap.RIGHT_CLIMBER_MOTOR_ID, RobotMap.CANIVORE_NAME);

        leftClimberMotor.setInverted(false);
        rightClimberMotor.setFollower(RobotMap.LEFT_CLIMBER_MOTOR_ID, true);

        leftClimberMotor.setBrake();
        rightClimberMotor.setBrake();

        leftClimberMotor.setStatorCurrentLimit(ClimberConstants.kClimberStatorCurrentLimit);
        rightClimberMotor.setStatorCurrentLimit(ClimberConstants.kClimberStatorCurrentLimit);
        leftClimberMotor.setSupplyCurrentLimit(ClimberConstants.kClimberSupplyCurrentLimit);
        rightClimberMotor.setSupplyCurrentLimit(ClimberConstants.kClimberSupplyCurrentLimit);
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
        leftClimberMotor.setPercentOutput(speed);
    }

    /**
     * @return returns left climber supply current draw (amps)
     */
    public double getLeftClimberSupplyCurrent() {
        return leftClimberMotor.getSupplyCurrent();
    }

    /**
     * @return returns right climber supply current draw (amps)
     */
    public double getRightClimberSupplyCurrent() {
        return rightClimberMotor.getSupplyCurrent();
    }

    /**
     * @return returns left climber stator current draw (amps)
     */
    public double getLeftClimberStatorCurrent() {
        return leftClimberMotor.getStatorCurrent();
    }

    /**
     * @return returns right climber stator current draw (amps)
     */
    public double getRightClimberStatorCurrent() {
        return rightClimberMotor.getStatorCurrent();
    }

    /**
     * @return returns temperature of leftClimberMotor in celcius
     */
    public double getLeftClimberTemperature() {
        return leftClimberMotor.getMotorTemperature();
    }

    /**
     * @return returns temperature of rightClimberMotor in celcius
     */
    public double getRightClimberTemperature() {
        return rightClimberMotor.getMotorTemperature();
    }

    /**
     * @return returns position reading of leftClimberMotor encoder (mechanism rotations)
     */
    public double getLeftClimberPosition() {
        return leftClimberMotor.getPosition();
    }

    /**
     * @return returns position reading of rightClimberMotor encoder (mechanism rotations)
     */
    public double getRightClimberPosition() {
        return rightClimberMotor.getPosition();
    }
}
