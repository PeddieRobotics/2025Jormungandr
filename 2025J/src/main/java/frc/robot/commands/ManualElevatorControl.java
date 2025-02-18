package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.utils.OperatorOI;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualElevatorControl extends Command {

    private Elevator elevator;
    private OperatorOI operatorOI;

    public ManualElevatorControl() {
        elevator = Elevator.getInstance();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        operatorOI = OperatorOI.getInstance();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.setElevatorPercentOutput(operatorOI.getForward());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}