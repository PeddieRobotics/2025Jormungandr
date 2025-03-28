package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightFrontRight extends Limelight {
    private static LimelightFrontRight llFrontRight;

    private LimelightFrontRight() {
        super(CameraConstants.kFrontRightCamName, false);
    }

    public static LimelightFrontRight getInstance() {
        if (llFrontRight == null)
            llFrontRight = new LimelightFrontRight();
        return llFrontRight;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}