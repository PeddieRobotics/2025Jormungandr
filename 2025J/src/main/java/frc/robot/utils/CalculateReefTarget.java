package frc.robot.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.Constants.AlignmentConstants;

class IDVectorPair {
    public int id;
    public Translation2d vector;
    public IDVectorPair(int id, Translation2d vector) {
        this.id = id;
        this.vector = vector;
    }
    public String toString() {
        return id + ": " + vector.getNorm();
    }
}

public class CalculateReefTarget {
    private static double cosineSimilarity(Translation2d a, Translation2d b) {
        return (a.getX() * b.getX() + a.getY() * b.getY()) / (a.getNorm() * b.getNorm());
    }

    public static int calculateTargetID() {    
        // calculate current odometry pose
        Translation2d odometryPose = Drivetrain.getInstance().getPose().getTranslation();
        List<IDVectorPair> robotToTag = new ArrayList<>();
        
        // BLUE:
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            for (int i = 17; i <= 22; i++) {
                Translation2d tagPose = Limelight.getAprilTagPose(i).getTranslation();
                robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
            }
        }
        // RED:
        else {
            for (int i = 6; i <= 11; i++) {
                Translation2d tagPose = Limelight.getAprilTagPose(i).getTranslation();
                robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
            }
        }
        
        // sort list in ascending order of vector magnitude / robot distance to tag
        Collections.sort(robotToTag, (o1, o2) -> (
            ((Double) o1.vector.getNorm()).compareTo(o2.vector.getNorm())
        ));
        
        // closest and second closest tag IDs
        int tag0id = robotToTag.get(0).id;
        int tag1id = robotToTag.get(1).id;

        SmartDashboard.putNumber("tag 0 id", tag0id);
        SmartDashboard.putNumber("tag 1 id", tag1id);
        SmartDashboard.putNumber("tag 0 distance", robotToTag.get(0).vector.getNorm());
        SmartDashboard.putNumber("tag 1 distance", robotToTag.get(1).vector.getNorm());

        // when you're very close to the reef
        if (robotToTag.get(0).vector.getNorm() <= AlignmentConstants.kDefaultToClosestDistance) {
            // return tag/side that requires the lowest rotation
            double gyro = Drivetrain.getInstance().getHeading();
            double error0 = Math.abs(AlignmentConstants.kReefDesiredAngle.get(tag0id) - gyro);
            double error1 = Math.abs(AlignmentConstants.kReefDesiredAngle.get(tag1id) - gyro);
            SmartDashboard.putNumber("tag 0 gyro error", error0);
            SmartDashboard.putNumber("tag 1 gyro error", error1);
            return error0 <= error1 ? tag0id : tag1id;
        }
        
        // if closest is significantly closer than second
        if (robotToTag.get(1).vector.getNorm() - robotToTag.get(0).vector.getNorm() >= 0.25)
            return tag0id;

        Translation2d robotMovement = Drivetrain.getInstance().getCurrentMovement();
        // not robot movement, simply move to closest tag
        if (robotMovement.getNorm() == 0)
            return tag0id;

        // return tag with highest cosine similarity / lowest angle
        double similar0 = cosineSimilarity(robotToTag.get(0).vector, robotMovement);
        double similar1 = cosineSimilarity(robotToTag.get(1).vector, robotMovement);
        SmartDashboard.putNumber("tag 0 similarity", similar0);
        SmartDashboard.putNumber("tag 1 similarity", similar1);
        return similar0 >= similar1 ? tag0id : tag1id;
    }
}
