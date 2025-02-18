package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.utils.OperatorOI;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualArmControl extends Command {

    private Arm arm;
    private OperatorOI operatorOI;

    public ManualArmControl() {
        arm = Arm.getInstance();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        operatorOI = OperatorOI.getInstance();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        arm.setArmPercentOutput(operatorOI.getForward());
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