package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.OperatorOI;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualElevatorControl extends Command {

    private Elevator elevator;
    private OperatorOI operatorOI;

    public ManualElevatorControl() {
        elevator = Elevator.getInstance();
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Superstructure.getInstance().setManualControl(true);
        Superstructure.getInstance().requestState(SuperstructureState.STOW);
        operatorOI = OperatorOI.getInstance();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.setElevatorPercentOutput(0.5 * operatorOI.getRightForward());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Superstructure.getInstance().setManualControl(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}