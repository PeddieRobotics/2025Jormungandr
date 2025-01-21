package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class Claw extends SubsystemBase{

    private static Claw claw;
    private Kraken clawMotor;
    
    public Claw() {
        clawMotor = new Kraken(RobotMap.CLAW_MOTOR_ID, RobotMap.CANIVORE_NAME);

        clawMotor.setInverted(false);
        clawMotor.setSupplyCurrentLimit(ClawConstants.kClawCurrentLimit);
        clawMotor.setBrake();

        SmartDashboard.putBoolean("hasGamePiece", false);
    }

    public static Claw getInstance(){
        if (claw == null){
            claw = new Claw();
        }
        return claw;
    }

    public double getDistanceSensor(){
        // TODO: implement sensor logic
        return 0.0;
    }

    public void setClaw(double speed){
        clawMotor.setMotor(speed);
    }

    public void intakePiece(){
        setClaw(ClawConstants.kClawIntakeSpeed);
    }

    public void outtakePiece(){
        setClaw(ClawConstants.kClawOuttakeSpeed);
    }


}
