package frc.robot.subsystems;

import frc.robot.utils.Constants.LimelightConstants;

public class LimelightFrontLeft extends Limelight {
    private static LimelightFrontLeft limelightFrontLeft;

    public LimelightFrontLeft() {
        super("limelight-front-left",
            LimelightConstants.kLimelightFrontLeftHeight,
            LimelightConstants.kLimelightFrontLeftPanningAngle,
            LimelightConstants.kLimelightFrontLeftPipeline
        );
        
    }

    public static LimelightFrontLeft getInstance() {
        if (limelightFrontLeft == null)
            limelightFrontLeft = new LimelightFrontLeft();
        return limelightFrontLeft;
    }


    @Override
    public void periodic() {
        updateRollingAverages();
    }
}
