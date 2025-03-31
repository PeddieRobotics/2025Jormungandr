package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightBackTop extends Limelight {
    private static LimelightBackTop llBackTop;

    private LimelightBackTop() {
        super(CameraConstants.kBackTopCamName, true);
    }

    public static LimelightBackTop getInstance() {
        if (llBackTop == null)
            llBackTop = new LimelightBackTop();
        return llBackTop;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}