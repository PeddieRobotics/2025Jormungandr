package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class PVBack extends PhotonVision {
    private static PVBack pvBack;

    private PVBack() {
        super(
            CameraConstants.kBackCamName,
            CameraConstants.kBackCamForward,
            CameraConstants.kBackCamLeftOffset,
            CameraConstants.kBackCamUpOffset,
            CameraConstants.kBackCamPitchDeg,
            CameraConstants.kBackCamYawDeg
        );
    }

    public static PVBack getInstance() {
        if (pvBack == null)
            pvBack = new PVBack();
        return pvBack;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}