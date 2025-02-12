package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class PVLeft extends PhotonVision {
    private static PVLeft pvLeft;

    private PVLeft() {
        super(
            CameraConstants.kLeftCamName,
            CameraConstants.kLeftCamForward,
            CameraConstants.kLeftCamLeftOffset,
            CameraConstants.kLeftCamUpOffset,
            CameraConstants.kLeftCamPanningAngle
        );
    }

    public static PVLeft getInstance() {
        if (pvLeft == null)
            pvLeft = new PVLeft();
        return pvLeft;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}