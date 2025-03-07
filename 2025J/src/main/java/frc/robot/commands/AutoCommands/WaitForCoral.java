package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;

public class WaitForCoral extends Command {
    // private double startTime;
    public WaitForCoral() {
        // startTime = 0;
    }

    @Override
    public void initialize() {
        // startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return Claw.getInstance().bothCoralSensorsTriggered();// || Timer.getFPGATimestamp() - startTime >= 2.0;
    }
}
