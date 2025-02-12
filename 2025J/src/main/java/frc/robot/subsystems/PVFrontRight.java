package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class PVFrontRight extends PhotonVision {
    private static PVFrontRight pvFrontRight;

    private PVFrontRight() {
        super(
            CameraConstants.kFrontRightCamName,
            CameraConstants.kFrontRightCamForward,
            CameraConstants.kFrontRightCamLeftOffset,
            CameraConstants.kFrontRightCamUpOffset,
            CameraConstants.kFrontRightCamPanningAngle
        );
    }

    public static PVFrontRight getInstance() {
        if (pvFrontRight == null)
            pvFrontRight = new PVFrontRight();
        return pvFrontRight;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}