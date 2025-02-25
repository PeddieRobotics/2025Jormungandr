package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightBack extends Limelight {
    private static LimelightBack llBack;

    private LimelightBack() {
        super(
            CameraConstants.kBackCamName,
            CameraConstants.kBackCamForward,
            CameraConstants.kBackCamLeftOffset,
            CameraConstants.kBackCamUpOffset,
            CameraConstants.kBackCamPitchDeg,
            CameraConstants.kBackCamYawDeg
        );
    }

    public static LimelightBack getInstance() {
        if (llBack == null)
            llBack = new LimelightBack();
        return llBack;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}