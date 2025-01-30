package frc.robot.utils;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HPIntake;
import frc.robot.subsystems.Superstructure;

public class Logger {
    private static Logger instance;
    private Drivetrain drivetrain;
    private Arm arm;
    private Autonomous autonomous;
    private Claw claw;
    private Elevator elevator;
    private HPIntake hpIntake;
    private Superstructure superstructure;
    private Climber climber;
    private DataLog log = DataLogManager.getLog();
    private DoubleLogEntry gyroAngleEntry, driveTrainSpeed, driveTrainAngle, driveTrainX, driveTrainY, driveTrainXVel,
            driveTrainYVel, driveTrainXAcc, driveTrainYAcc, driveTrainAngleVel, driveTrainAngleAcc, armAngle, armSpeed,
            armAcc, armPosition, armVelocity, clawAngle, clawAcc, clawPosition, clawVelocity, elevatorPosition,
            elevatorVelocity, elevatorAcc, elevatorSpeed, hpIntakePosition, hpIntakeVelocity, hpIntakeAcc,
            superstructurePosition, superstructureVelocity, superstructureAcc, leftClimberSupplyCurrent,
            leftClimberStatorCurrent, leftClimberTemperature, leftClimberPosition, rightClimberSupplyCurrent,
            rightClimberStatorCurrent, rightClimberTemperature, rightClimberPosition;

    public static Logger getInstance() {
        if (instance == null) {
            instance = new Logger();
        }
        return instance;
    }

    public Logger() {
        drivetrain = Drivetrain.getInstance();
        // arm = Arm.getInstance();
        autonomous = Autonomous.getInstance();
        // claw = Claw.getInstance();
        // elevator = Elevator.getInstance();
        // hpIntake = HPIntake.getInstance();
        // superstructure = Superstructure.getInstance();

        // climber = Climber.getInstance();
        leftClimberPosition = new DoubleLogEntry(log, "/Climber/Left Climber Position");
    }

    public void updateLogs(){
       // leftClimberPosition.append(climber.getLeftClimberPosition());
    }
}
