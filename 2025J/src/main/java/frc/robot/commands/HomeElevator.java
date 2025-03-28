package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.ElevatorConstants;

/** An example command that uses an example subsystem. */
public class HomeElevator extends Command {

    private Elevator elevator;
    private Superstructure superstructure;
    private double initialTime, timeDelta;
    

    public HomeElevator() {
        elevator = Elevator.getInstance();
        superstructure = Superstructure.getInstance();

        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();
        elevator.disableSoftLimits();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        timeDelta = Timer.getFPGATimestamp() - initialTime;
        elevator.setElevatorPercentOutput(ElevatorConstants.kHomeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorNeutralMode();
        
        elevator.resetElevatorPosition();
        elevator.resetElevatorCANCoder();
        elevator.enableSoftLimits();
        superstructure.requestState(SuperstructureState.HP_INTAKE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timeDelta > ElevatorConstants.kMinHomeTime && Math.abs(elevator.getCANCoderVelocity()) < ElevatorConstants.kHomingSpeedThreshold;
    }
}