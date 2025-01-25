package frc.robot.subsystems;

import frc.robot.utils.Constants;

public class LimelightFrontLeft extends Limelight {
    private static LimelightFrontLeft limelightFrontLeft;

    public LimelightFrontLeft() {
        super("limelight-front-left",
            Constants.LimelightConstants.kLimelightFrontLeftHeight,
            Constants.LimelightConstants.kLimelightFrontLeftPanningAngle,
            0
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
