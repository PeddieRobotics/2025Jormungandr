package frc.robot.subsystems;

import frc.robot.utils.Constants;

public class LimelightFrontRight extends Limelight {
    private static LimelightFrontRight limelightFrontRight;

    public LimelightFrontRight() {
        super("limelight-front-right",
            Constants.LimelightConstants.kLimelightFrontRightHeight,
            Constants.LimelightConstants.kLimelightFrontRightPanningAngle,
            0
        );
        
    }

    public static LimelightFrontRight getInstance() {
        if (limelightFrontRight == null)
            limelightFrontRight = new LimelightFrontRight();
        return limelightFrontRight;
    }


    @Override
    public void periodic() {
        updateRollingAverages();
    }
}
