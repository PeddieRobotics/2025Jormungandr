package frc.robot.subsystems;

import frc.robot.utils.Constants.LimelightConstants;

public class LimelightFrontRight extends Limelight {
    private static LimelightFrontRight limelightFrontRight;

    public LimelightFrontRight() {
        super("limelight-front-right",
            LimelightConstants.kLimelightFrontRightHeight,
            LimelightConstants.kLimelightFrontRightPanningAngle,
            LimelightConstants.kLimelightFrontRightPipeline
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
