package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightClimber extends Limelight {
    private static LimelightClimber llClimber;

    private LimelightClimber() {
        super(CameraConstants.kClimberCamName, false);
    }

    public static LimelightClimber getInstance() {
        if (llClimber == null)
            llClimber = new LimelightClimber();
        return llClimber;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}