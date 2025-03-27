package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.HPIntake;
import frc.robot.utils.Logger;

public class DeployClimber extends Command {

    private Climber climber;
    private HPIntake hpIntake;

    public DeployClimber() {
        climber = Climber.getInstance();
        hpIntake = HPIntake.getInstance();

        addRequirements(climber, hpIntake);
    }

    @Override
    public void initialize() {
        Logger.getInstance().logEvent("Deploy Climber", true);
    }

    @Override
    public void execute() {
        hpIntake.retractLinearActuator();
        climber.deployClimber();
    }

    @Override
    public void end(boolean interrupted) {
        Logger.getInstance().logEvent("Deploy Climber", false);
    }

    @Override
    public boolean isFinished() {
        return climber.climbDeployed();
    }

}
