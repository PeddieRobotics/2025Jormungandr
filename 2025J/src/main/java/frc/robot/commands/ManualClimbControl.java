package frc.robot.commands;

import frc.robot.subsystems.Climber;
import frc.robot.utils.OperatorOI;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualClimbControl extends Command {

    private Climber climber;
    private OperatorOI operatorOI;

    public ManualClimbControl() {
        climber = Climber.getInstance();
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        operatorOI = OperatorOI.getInstance();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.setSpeed(operatorOI.getForward());
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