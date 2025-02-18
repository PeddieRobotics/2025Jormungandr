package frc.robot.commands.ReefCommands;

//import frc.robot.utils.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.DriverOI;

public class OrbitReef extends Command {

    private Drivetrain drivetrain;
    private DriverOI oi;

    private double P = 0.12, I = 0, D = 0, FF = 0.0;

    private double turnThreshold;
    private PIDController turnPIDController;
    private double currentHeading, initialHeading;

    private double setpoint;

    private double reefCenterX;
    private double reefCenterY;

    public OrbitReef() { //center of the reef is (4.5, 4)!!!
        drivetrain = Drivetrain.getInstance();
        
        SmartDashboard.putNumber("P turn pid orbitReef", P);
        SmartDashboard.putNumber("I turn pid orbitReef", I);
        SmartDashboard.putNumber("D turn pid orbitReef", D);
        SmartDashboard.putNumber("FF turn pid orbitReef", FF);

        turnPIDController = new PIDController(P, I, D);
        turnPIDController.enableContinuousInput(-180, 180); //wrap around values 

        addRequirements(drivetrain);
    }

    
    @Override
    public void initialize() {
        oi = DriverOI.getInstance();

        if (DriverStation.getAlliance().get() == Alliance.Red){
            reefCenterX = FieldConstants.reefCenterXRed;
            reefCenterY = FieldConstants.reefCenterYRed;
        }
        else {
            reefCenterX = FieldConstants.reefCenterXBlue;
            reefCenterY = FieldConstants.reefCenterYBlue;
        }
    }

    @Override
    public void execute() {
        P = SmartDashboard.getNumber("P turn pid orbitReef", P);
        I = SmartDashboard.getNumber("I turn pid orbitReef", I);
        D = SmartDashboard.getNumber("D turn pid orbitReef", D);
        FF = SmartDashboard.getNumber("FF turn pid orbitReef", FF);

        turnPIDController.setPID(P, I, D);
        
        currentHeading = drivetrain.getHeading();

        double currentHeading = drivetrain.getHeading();
        double poseX = drivetrain.getPose().getX();
        double poseY = drivetrain.getPose().getY();
        setpoint = Math.atan2(reefCenterY - poseY, reefCenterX - poseX);
        setpoint = Math.toDegrees(setpoint);

        double turnToReef = turnPIDController.calculate(currentHeading, setpoint);
        //turnToReef + FF * Math.signum(turnToReef)
        drivetrain.drive(oi.getSwerveTranslation(), turnToReef + FF * Math.signum(turnToReef), true, new Translation2d(0, 0));

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return Math.abs(Math.abs(currentHeading) - setpoint) < turnThreshold;
    }
}