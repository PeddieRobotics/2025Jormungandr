package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.Constants.ClimberConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.LiveData;
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

    public void setSpeed(double speed) {
        leftClimberMotor.setPercentOutput(speed);
    }

    public double getLeftClimberSupplyCurrent() {
        return leftClimberMotor.getSupplyCurrent();
    }

    public double getRightClimberSupplyCurrent() {
        return rightClimberMotor.getSupplyCurrent();
    }

    public double getLeftClimberStatorCurrent() {
        return leftClimberMotor.getStatorCurrent();
    }

    public double getRightClimberStatorCurrent() {
        return rightClimberMotor.getStatorCurrent();
    }

    public double getLeftClimberTemperature() {
        return leftClimberMotor.getMotorTemperature();
    }

    public double getRightClimberTemperature() {
        return rightClimberMotor.getMotorTemperature();
    }

    public double getLeftClimberPosition() {
        return leftClimberMotor.getPosition();
    }

    public double getRightClimberPosition() {
        return rightClimberMotor.getPosition();
    }
}
