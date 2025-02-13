package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class PVFrontLeft extends PhotonVision {
    private static PVFrontLeft pvFrontLeft;

    private PVFrontLeft() {
        super(
            CameraConstants.kFrontLeftCamName,
            CameraConstants.kFrontLeftCamForward,
            CameraConstants.kFrontLeftCamLeftOffset,
            CameraConstants.kFrontLeftCamUpOffset,
            CameraConstants.kFrontLeftCamPitchDeg,
            CameraConstants.kFrontLeftCamYawDeg
        );
    }

    public static PVFrontLeft getInstance() {
        if (pvFrontLeft == null)
            pvFrontLeft = new PVFrontLeft();
        return pvFrontLeft;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}