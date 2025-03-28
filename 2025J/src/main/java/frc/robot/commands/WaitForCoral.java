package frc.robot.commands;

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
        // return Claw.getInstance().bothCoralSensorsTriggered();// || Timer.getFPGATimestamp() - startTime >= 2.0;
        return Claw.getInstance().getTopSensor();// || Timer.getFPGATimestamp() - startTime >= 2.0;
    }
}
