package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightClimber extends Limelight {
    private static LimelightClimber llLeft;

    private LimelightClimber() {
        super(
            CameraConstants.kLeftCamName,
            CameraConstants.kLeftCamUpOffset,
            CameraConstants.kLeftCamPitchDeg,
            false
        );
    }

    public static LimelightClimber getInstance() {
        if (llLeft == null)
            llLeft = new LimelightClimber();
        return llLeft;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}