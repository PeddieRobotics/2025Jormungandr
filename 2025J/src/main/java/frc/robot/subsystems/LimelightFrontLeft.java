package frc.robot.subsystems;

import frc.robot.utils.Constants;

public class LimelightFrontLeft extends Limelight {
    private static LimelightBack limelightFrontLeft;

    public LimelightFrontLeft() {
        super("limelight-front-left",
            Constants.LimelightConstants.kLimelightFrontLeftHeight,
            Constants.LimelightConstants.kLimelightFrontLeftPanningAngle,
            0
        );
        
    }

    public static LimelightBack getInstance() {
        if (limelightFrontLeft == null)
            limelightFrontLeft = new LimelightBack();
        return limelightFrontLeft;
    }


    @Override
    public void periodic() {
        updateRollingAverages();
    }
}
