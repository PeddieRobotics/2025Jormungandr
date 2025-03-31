package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightBackBottom extends Limelight {
    private static LimelightBackBottom llBackBottom;

    private LimelightBackBottom() {
        super(CameraConstants.kBackBottomCamName, true);
    }

    public static LimelightBackBottom getInstance() {
        if (llBackBottom == null)
            llBackBottom = new LimelightBackBottom();
        return llBackBottom;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}