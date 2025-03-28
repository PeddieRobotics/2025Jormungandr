package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class CalculateHPTarget {
    public static int calculateTargetID() {
        int station0, station1;
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            station0 = 12;
            station1 = 13;
        }
        else {
            station0 = 1;
            station1 = 2;
        }
        
        // double gyro = DriverStation.isAutonomous() ? drivetrain.getHeadingBlueForceAdjust() : drivetrain.getHeadingBlue();

        // double heading0 = Limelight.getAprilTagPose(station0).getRotation().getDegrees();
        // double heading1 = Limelight.getAprilTagPose(station1).getRotation().getDegrees();

        // double error0 = Math.abs(heading0 - gyro);
        // double error1 = Math.abs(heading1 - gyro);

        // return error0 <= error1 ? station0 : station1;

        Pose2d position0 = Limelight.getAprilTagPose(station0);
        Pose2d position1 = Limelight.getAprilTagPose(station1);

        Pose2d odometry = Drivetrain.getInstance().getPose();

        double distance0 = odometry.minus(position0).getTranslation().getNorm();
        double distance1 = odometry.minus(position1).getTranslation().getNorm();

        return distance0 <= distance1 ? station0 : station1;
    }
}
