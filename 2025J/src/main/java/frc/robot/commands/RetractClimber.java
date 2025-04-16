package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.utils.Logger;

public class RetractClimber extends Command {

    private Climber climber;

    public RetractClimber() {
        climber = Climber.getInstance();
    }

    @Override
    public void initialize() {
        Logger.getInstance().logEvent("Retract Climber", true);
    }

    @Override
    public void execute() {
        climber.retractClimber();
    }

    @Override
    public void end(boolean interrupted) {
        Logger.getInstance().logEvent("Retract Climber", false);
    }

    @Override
    public boolean isFinished() {
        return climber.climbRetracted();
    }

}
