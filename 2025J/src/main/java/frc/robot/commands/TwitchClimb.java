package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TwitchClimb extends Command {
    private Drivetrain drivetrain;
    private double currentTime;
    private double startTime;
    private double totalTime;
    private double percentOutput;
    private boolean direction;

    public TwitchClimb(boolean direction) {
        this.direction = direction;

        drivetrain = Drivetrain.getInstance();

        totalTime = 0.1;
        percentOutput = 0.9;

        SmartDashboard.putNumber("Twitch: Time", totalTime);
        SmartDashboard.putNumber("Twitch: Percent Rotation", percentOutput);

        currentTime = 0;
        startTime = 9999;
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        totalTime = SmartDashboard.getNumber("Twitch: Time", totalTime);
        percentOutput = SmartDashboard.getNumber("Twitch: Percent Rotation", percentOutput);
        percentOutput *= direction ? 1 : -1;

        currentTime = Timer.getFPGATimestamp();
        if (currentTime - startTime > totalTime)
            return;

        drivetrain.drive(new Translation2d(0, 0), percentOutput, false, null);
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return currentTime - startTime > totalTime;
    }
}
