package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightBackClimb extends Limelight {
    private static LimelightBackClimb llBackClimb;

    private LimelightBackClimb() {
        super(
            CameraConstants.kBackClimbCamName,
            CameraConstants.kBackClimbCamUpOffset,
            CameraConstants.kBackClimbCamPitchDeg,
            false
        );
    }

    public static LimelightBackClimb getInstance() {
        if (llBackClimb == null)
            llBackClimb = new LimelightBackClimb();
        return llBackClimb;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}