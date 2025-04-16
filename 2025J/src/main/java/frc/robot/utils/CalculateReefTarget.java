package frc.robot.utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

class Point {
    public double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(Translation2d t2d) {
        this.x = t2d.getX();
        this.y = t2d.getY();
    }
}

public class CalculateReefTarget {
    private static Drivetrain drivetrain;
    private static List<Point> badHexagonPointsRed, badHexagonPointsBlue;

    private static double cosineSimilarity(Translation2d a, Translation2d b) {
        return (a.getX() * b.getX() + a.getY() * b.getY()) / (a.getNorm() * b.getNorm());
    }

    public static void initBlue() {
        drivetrain = Drivetrain.getInstance();

        Pose2d tag18 = Limelight.getAprilTagPose(18);
        Pose2d tag21 = Limelight.getAprilTagPose(21);
        Point reefCenter = new Point(
            (tag18.getX() + tag21.getX()) / 2,
            (tag18.getY() + tag21.getY()) / 2
        );

        /*
         * center to tag = 32.75
         * divide reef hexagon into 6 hexagons: each are equilateral
         *        /|
         *       / |
         *      /  |
         *     /   |
         *  a /    | 32.75
         *   /     |
         *  /      |
         * / 60d   |
         * ----------
         *    sin(60d) = sqrt(3)/2 = 32.75/a
         * => a = 2*32.75 / sqrt(3) = 37.816
         */

        double reefCornerToCenter = Units.inchesToMeters(2 * 32.75 / Math.sqrt(3));
        reefCornerToCenter += AlignmentConstants.kBadHexagonSize;

        badHexagonPointsBlue = new ArrayList<>();
        for (int i = 0; i < 360; i += 60) {
            badHexagonPointsBlue.add(new Point(
                reefCenter.x + reefCornerToCenter * Math.cos(Math.toRadians(i)),
                reefCenter.y + reefCornerToCenter * Math.sin(Math.toRadians(i))
            ));
        }
    }

    public static void initRed() {
        drivetrain = Drivetrain.getInstance();

        Pose2d tag7 = Limelight.getAprilTagPose(7);
        Pose2d tag10 = Limelight.getAprilTagPose(10);
        Point reefCenter = new Point(
            (tag7.getX() + tag10.getX()) / 2,
            (tag7.getY() + tag10.getY()) / 2
        );

        double reefCornerToCenter = Units.inchesToMeters(2 * 32.75 / Math.sqrt(3));
        reefCornerToCenter += AlignmentConstants.kBadHexagonSize;

        badHexagonPointsRed = new ArrayList<>();
        for (int i = 0; i < 360; i += 60) {
            badHexagonPointsRed.add(new Point(
                reefCenter.x + reefCornerToCenter * Math.cos(Math.toRadians(i)),
                reefCenter.y + reefCornerToCenter * Math.sin(Math.toRadians(i))
            ));
        }
    }

    // Solution 2: https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
    private static double angle2d(double x1, double y1, double x2, double y2) {
        double dtheta, theta1, theta2;
        theta1 = Math.atan2(y1,x1);
        theta2 = Math.atan2(y2,x2);
        dtheta = theta2 - theta1;
        while (dtheta > Math.PI)
            dtheta -= 2 * Math.PI;
        while (dtheta < -Math.PI)
            dtheta += 2 * Math.PI;
        return dtheta;
    }
    private static boolean insideBadHexagonBlue(Point p) {
        double angle = 0;
        Point p1 = new Point(0, 0);
        Point p2 = new Point(0, 0);

        for (int i = 0; i < 6; i++) {
            p1.x = badHexagonPointsBlue.get(i).x - p.x;
            p1.y = badHexagonPointsBlue.get(i).y - p.y;
            p2.x = badHexagonPointsBlue.get((i + 1) % 6).x - p.x;
            p2.y = badHexagonPointsBlue.get((i + 1) % 6).y - p.y;
            angle += angle2d(p1.x, p1.y, p2.x, p2.y);
        }

        return Math.abs(angle) >= Math.PI;
    }
    private static boolean insideBadHexagonRed(Point p) {
        double angle = 0;
        Point p1 = new Point(0, 0);
        Point p2 = new Point(0, 0);

        for (int i = 0; i < 6; i++) {
            p1.x = badHexagonPointsRed.get(i).x - p.x;
            p1.y = badHexagonPointsRed.get(i).y - p.y;
            p2.x = badHexagonPointsRed.get((i + 1) % 6).x - p.x;
            p2.y = badHexagonPointsRed.get((i + 1) % 6).y - p.y;
            angle += angle2d(p1.x, p1.y, p2.x, p2.y);
        }

        return Math.abs(angle) >= Math.PI;
    }

    private static boolean isInBlueSide() {
        return drivetrain.getPose().getX() < 17.55 / 2;
    }

    public static int calculateTargetID() {
        // calculate current odometry pose
        Translation2d odometryPose = drivetrain.getPose().getTranslation();
        List<IDVectorPair> robotToTag = new ArrayList<>();

        // // BLUE:
        // if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        //     for (int i = 17; i <= 22; i++) {
        //         Translation2d tagPose = Limelight.getAprilTagPose(i).getTranslation();
        //         robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
        //     }
        // }
        // // RED:
        // else {
        //     for (int i = 6; i <= 11; i++) {
        //         Translation2d tagPose = Limelight.getAprilTagPose(i).getTranslation();
        //         robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
        //     }
        // }
        
        if (isInBlueSide()) {
            for (int i = 17; i <= 22; i++) {
                Translation2d tagPose = Limelight.getAprilTagPose(i).getTranslation();
                robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
            }
        } else {
            for (int i = 6; i <= 11; i++) {
                Translation2d tagPose = Limelight.getAprilTagPose(i).getTranslation();
                robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
            }
        }

        // sort list in ascending order of vector magnitude / robot distance to tag
        Collections.sort(robotToTag, (o1, o2) -> (((Double) o1.vector.getNorm()).compareTo(o2.vector.getNorm())));

        // closest and second closest tag IDs
        int tag0id = robotToTag.get(0).id;
        int tag1id = robotToTag.get(1).id;

        Point point = new Point(odometryPose);
        boolean isInBadHexagon = insideBadHexagonBlue(point) || insideBadHexagonRed(point);

        // SmartDashboard.putBoolean("is inside bad hexagon", isInBadHexagon);

        // SmartDashboard.putNumber("tag 0 distance", robotToTag.get(0).vector.getNorm());
        // SmartDashboard.putNumber("tag 1 distance", robotToTag.get(1).vector.getNorm());
        // SmartDashboard.putNumber("tag 0 id", tag0id);
        // SmartDashboard.putNumber("key 1 id", tag1id);

        double tag0neededAngle = AlignmentConstants.kReefDesiredAngle.get(tag0id);
        double tag0gyroError = drivetrain.getHeadingBlue() - tag0neededAngle;
        if (tag0gyroError < -180)
            tag0gyroError += 360;
        if (tag0gyroError > 180)
            tag0gyroError -= 360;
        tag0gyroError = Math.abs(tag0gyroError);

        if (isInBadHexagon)
            return tag0gyroError < AlignmentConstants.kInsideBadAngleTolerance ? tag0id : 0;

        // TODO: tune 0.25 value
        if (robotToTag.get(1).vector.getNorm() - robotToTag.get(0).vector.getNorm() >= 0.25)
            return tag0gyroError < AlignmentConstants.kOutsideBadAngleTolerance ? tag0id : 0;

        Translation2d robotMovement = drivetrain.getCurrentMovement();

        int bestTag;
        if (robotMovement.getNorm() == 0)
            bestTag = tag0id;
        else {
            double similar0 = cosineSimilarity(robotToTag.get(0).vector, robotMovement);
            double similar1 = cosineSimilarity(robotToTag.get(1).vector, robotMovement);
            // SmartDashboard.putNumber("tag 0 similarity", similar0);
            // SmartDashboard.putNumber("tag 1 similarity", similar1);
            bestTag = similar0 >= similar1 ? tag0id : tag1id;
        }

        double bestTagNeededAngle = AlignmentConstants.kReefDesiredAngle.get(bestTag);
        double bestTagError = drivetrain.getHeadingBlue() - bestTagNeededAngle;
        if (bestTagError < -180)
            bestTagError += 360;
        if (bestTagError > 180)
            bestTagError -= 180;
        bestTagError = Math.abs(tag0gyroError);

        return bestTagError < AlignmentConstants.kOutsideBadAngleTolerance ? bestTag : 0;
    }
}
