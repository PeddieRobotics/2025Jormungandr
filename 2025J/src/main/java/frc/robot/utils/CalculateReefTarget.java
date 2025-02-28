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
        /*
         * desired target algorithm
         * 1. calculate distance from current odometry to each tag and order list
         * 2. if lowest is "significantly lower" than second lowest, use lowest (END)
         * 3. find the robot's current movement vector
         * 4. find cosine similarity between robot movement vector and vector
         *      from robot to each of the top 2 tags
         * 5. use tag with lower cosine similarity
         */

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
        
        Collections.sort(robotToTag, (o1, o2) -> (
            ((Double) o1.vector.getNorm()).compareTo(o2.vector.getNorm())
        ));

        SmartDashboard.putNumber("similar 0 tag", robotToTag.get(0).id);
        SmartDashboard.putNumber("similar 1 tag", robotToTag.get(1).id);
        SmartDashboard.putNumber("similar 0 distance", robotToTag.get(0).vector.getNorm());
        SmartDashboard.putNumber("similar 1 distance", robotToTag.get(1).vector.getNorm());

        if (robotToTag.get(0).vector.getNorm() <= AlignmentConstants.kDefaultToClosestDistance)
            return robotToTag.get(0).id;
        
        int desiredTarget;
        if (robotToTag.get(1).vector.getNorm() - robotToTag.get(0).vector.getNorm() >= 0.25)
            desiredTarget = robotToTag.get(0).id;
        else {
            Translation2d robotMovement = Drivetrain.getInstance().getCurrentMovement();
            if (robotMovement.getNorm() == 0)
                desiredTarget = robotToTag.get(0).id;
            else {
                double similar0 = cosineSimilarity(robotToTag.get(0).vector, robotMovement);
                double similar1 = cosineSimilarity(robotToTag.get(1).vector, robotMovement);
                desiredTarget = similar0 > similar1 ? robotToTag.get(0).id : robotToTag.get(1).id;
                SmartDashboard.putNumber("similar 0 similarity", similar0);
                SmartDashboard.putNumber("similar 1 similarity", similar1);
            }
        }

        return desiredTarget;
    }
}
