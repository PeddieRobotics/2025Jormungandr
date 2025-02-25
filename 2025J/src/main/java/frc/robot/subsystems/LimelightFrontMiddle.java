package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightFrontMiddle extends Limelight {
    private static LimelightFrontMiddle llFrontMiddle;

    private LimelightFrontMiddle() {
        super(
            CameraConstants.kFrontMiddleCamName,
            CameraConstants.kFrontMiddleCamForward,
            CameraConstants.kFrontMiddleCamLeftOffset,
            CameraConstants.kFrontMiddleCamUpOffset,
            CameraConstants.kFrontMiddleCamPitchDeg,
            CameraConstants.kFrontMiddleCamYawDeg
        );
    }

    public static LimelightFrontMiddle getInstance() {
        if (llFrontMiddle == null)
            llFrontMiddle = new LimelightFrontMiddle();
        return llFrontMiddle;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}