package frc.robot.subsystems;

import frc.robot.utils.Constants;

public class LimelightBack extends Limelight {
    private static LimelightBack limelightBack;

    public LimelightBack() {
        super(
            "limelight-back",
            Constants.LimelightConstants.kLimelightBackHeight,
            Constants.LimelightConstants.kLimelightBackPanningAngle,
            0
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
