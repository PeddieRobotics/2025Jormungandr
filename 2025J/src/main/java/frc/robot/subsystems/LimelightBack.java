package frc.robot.subsystems;

import frc.robot.utils.Constants.LimelightConstants;

public class LimelightBack extends Limelight {
    private static LimelightBack limelightBack;

    public LimelightBack() {
        super(
            "limelight-back",
            LimelightConstants.kLimelightBackHeight,
            LimelightConstants.kLimelightBackPanningAngle,
            LimelightConstants.kLimelightBackPipeline
        );
        
    }

    public static LimelightBack getInstance() {
        if (limelightBack == null)
            limelightBack = new LimelightBack();
        return limelightBack;
    }


    @Override
    public void periodic() {
        updateRollingAverages();
    }
}
