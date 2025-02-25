package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightLeft extends Limelight {
    private static LimelightLeft llLeft;

    private LimelightLeft() {
        super(
            CameraConstants.kLeftCamName,
            CameraConstants.kLeftCamForward,
            CameraConstants.kLeftCamLeftOffset,
            CameraConstants.kLeftCamUpOffset,
            CameraConstants.kLeftCamPitchDeg,
            CameraConstants.kLeftCamYawDeg
        );
    }

    public static LimelightLeft getInstance() {
        if (llLeft == null)
            llLeft = new LimelightLeft();
        return llLeft;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}