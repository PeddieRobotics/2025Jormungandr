package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class PVFrontMiddle extends PhotonVision {
    private static PVFrontMiddle pvFrontMiddle;

    private PVFrontMiddle() {
        super(
            CameraConstants.kFrontMiddleCamName,
            CameraConstants.kFrontMiddleCamForward,
            CameraConstants.kFrontMiddleCamLeftOffset,
            CameraConstants.kFrontMiddleCamUpOffset,
            CameraConstants.kFrontMiddleCamPanningAngle
        );
    }

    public static PVFrontMiddle getInstance() {
        if (pvFrontMiddle == null)
            pvFrontMiddle = new PVFrontMiddle();
        return pvFrontMiddle;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}