package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightFrontLeft extends Limelight {
    private static LimelightFrontLeft llFrontLeft;

    private LimelightFrontLeft() {
        super(
            CameraConstants.kFrontLeftCamName,
            CameraConstants.kFrontLeftCamForward,
            CameraConstants.kFrontLeftCamLeftOffset,
            CameraConstants.kFrontLeftCamUpOffset,
            CameraConstants.kFrontLeftCamPitchDeg,
            CameraConstants.kFrontLeftCamYawDeg
        );
    }

    public static LimelightFrontLeft getInstance() {
        if (llFrontLeft == null)
            llFrontLeft = new LimelightFrontLeft();
        return llFrontLeft;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}