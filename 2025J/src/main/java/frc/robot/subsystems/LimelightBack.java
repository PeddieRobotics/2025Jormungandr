package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightBack extends Limelight {
    private static LimelightBack llBack;

    private LimelightBack() {
        super(
            CameraConstants.kBackCamName,
            CameraConstants.kBackCamUpOffset,
            CameraConstants.kBackCamPitchDeg,
            false
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