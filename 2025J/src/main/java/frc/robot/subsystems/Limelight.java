package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.RollingAverage;

public abstract class Limelight extends SubsystemBase {
    /**
     * Algorithm from https://docs.limelightvision.io/en/latest/cs_estimating_distance.html Estimates
     * range to a target using the target's elevation. This method can produce more stable results
     * than SolvePNP when well tuned, if the full 6d robot pose is not required. Note that this method
     * requires the camera to have 0 roll (not be skewed clockwise or CCW relative to the floor), and
     * for there to exist a height differential between goal and camera. The larger this differential,
     * the more accurate the distance estimate will be.
     *
     * <p>Units can be converted using the {@link edu.wpi.first.math.util.Units} class.
     *
     * @param cameraHeightMeters The physical height of the camera off the floor in meters.
     * @param targetHeightMeters The physical height of the target off the floor in meters. This
     *     should be the height of whatever is being targeted (i.e. if the targeting region is set to
     *     top, this should be the height of the top of the target).
     * @param cameraPitchRadians The pitch of the camera from the horizontal plane in radians.
     *     Positive values up.
     * @param targetPitchRadians The pitch of the target in the camera's lens in radians. Positive
     *     values up.
     * @return The estimated distance to the target in meters.
     */
    public static double calculateDistanceToTargetMeters(
            double cameraHeightMeters,
            double targetHeightMeters,
            double cameraPitchRadians,
            double targetPitchRadians) {
        return (targetHeightMeters - cameraHeightMeters)
                / Math.tan(cameraPitchRadians + targetPitchRadians);
    }

    // public static enum PoseObservationType {
    //     MEGATAG_1,
    //     MEGATAG_2
    // }
    // public static record PoseObservation (
    //     double timestamp,
    //     Pose3d pose,
    //     double ambiguity,
    //     int tagCount,
    //     double averageTagDistance,
    //     PoseObservationType type
    // ) {}

    private static AprilTagFieldLayout aprilTagFieldLayout;

    private RollingAverage txAverage, tyAverage;
    private LinearFilter distTyFilter, distEstimatedPoseFilter;
    
    private String limelightName; 
    
    private double cameraUpOffset;
    private double cameraPitchRadians;
    
    // private DoubleArrayPublisher orientationPublisher;
    // private DoubleArraySubscriber megatag1Subscriber, megatag2Subscriber;
    // private PoseObservation[] poseObservationsMT2;
    
    private Field2d fieldMT1, fieldMT2;
    private StructPublisher<Pose2d> publisherMT1, publisherMT2;

    protected Limelight(String limelightName, double cameraUpOffset, double cameraPitchDegrees) {
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        
        this.limelightName = limelightName;
        this.cameraUpOffset = cameraUpOffset;
        this.cameraPitchRadians = Math.toRadians(cameraPitchDegrees);

        // var table = NetworkTableInstance.getDefault().getTable(limelightName);
        // orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        // megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        // megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

        txAverage = new RollingAverage();
        tyAverage = new RollingAverage();

        distTyFilter = LinearFilter.singlePoleIIR(0.24, 0.02);
        distEstimatedPoseFilter = LinearFilter.singlePoleIIR(0.24, 0.02);

        fieldMT1 = new Field2d();
        // fieldMT2 = new Field2d();
        SmartDashboard.putData(limelightName + " estimated pose (MT1)", fieldMT1);
        // SmartDashboard.putData(limelightName + " estimated pose (MT2)", fieldMT2);

        publisherMT1 = NetworkTableInstance.getDefault().getStructTopic(
            limelightName + " estimated pose for advantagescope (MT1)", Pose2d.struct
        ).publish();
        // publisherMT2 = NetworkTableInstance.getDefault().getStructTopic(
        //     limelightName + " estimated pose for advantagescope (MT2)", Pose2d.struct
        // ).publish();
    }
    
    @Override 
    public void periodic() {
        updateRollingAverages();

        Optional<Pose2d> estimatedPoseMT1 = getEstimatedPoseMT1();
        if (estimatedPoseMT1.isPresent()) {
            fieldMT1.setRobotPose(estimatedPoseMT1.get());
            publisherMT1.set(estimatedPoseMT1.get());
        }

        // Optional<PoseObservation> poseObservationMT2 = getEstimatedPoseMT2();
        // if (poseObservationMT2.isPresent()) {
        //     Pose2d estimatedPoseMT2 = poseObservationMT2.get().pose.toPose2d();
        //     fieldMT2.setRobotPose(estimatedPoseMT2);
        //     publisherMT2.set(estimatedPoseMT2);
        // }

        SmartDashboard.putNumber(limelightName + " Tx", getTx());
        SmartDashboard.putNumber(limelightName + " Ty", getTy());
        SmartDashboard.putNumber(limelightName + " distance (estimated pose)", getDistanceEstimatedPose());
        SmartDashboard.putNumber(limelightName + " distance (Ty)", getDistanceTy());
        SmartDashboard.putNumber(limelightName + " filtered distance (estimated pose)", getFilteredDistanceEstimatedPose());
        SmartDashboard.putNumber(limelightName + " filtered distance (Ty)", getFilteredDistanceTy());
        SmartDashboard.putNumber(limelightName + " number of tags seen", getNumberOfTagsSeen());
        SmartDashboard.putNumber(limelightName + " target ID", getTargetID());
        SmartDashboard.putBoolean(limelightName + " has target", hasTarget());

        // orientationPublisher.accept(new double[] { Drivetrain.getInstance().getHeading(), 0.0, 0.0, 0.0, 0.0, 0.0 });
        // NetworkTableInstance.getDefault().flush();
        
        // List<PoseObservation> poseObservations = new LinkedList<>();
        // for (var rawSample : megatag1Subscriber.readQueue()) {
        //     if (rawSample.value.length == 0)
        //         continue;
        //     poseObservations.add(
        //         new PoseObservation(
        //             // Timestamp, based on server timestamp of publish and latency
        //             rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

        //             // 3D pose estimate
        //             parsePose(rawSample.value),

        //             // Ambiguity, using only the first tag because ambiguity isn't
        //             // applicable for multitag
        //             rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

        //             // Tag count
        //             (int) rawSample.value[7],

        //             // Average tag distance
        //             rawSample.value[9],

        //             // Observation type
        //             PoseObservationType.MEGATAG_1
        //         )
        //     );
        // }
        // for (var rawSample : megatag2Subscriber.readQueue()) {
        //     if (rawSample.value.length == 0)
        //         continue;
        //     poseObservations.add(
        //         new PoseObservation(
        //             // Timestamp, based on server timestamp of publish and latency
        //             rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

        //             // 3D pose estimate
        //             parsePose(rawSample.value),

        //             // Ambiguity, zeroed because the pose is already disambiguated
        //             0.0,

        //             // Tag count
        //             (int) rawSample.value[7],

        //             // Average tag distance
        //             rawSample.value[9],

        //             // Observation type
        //             PoseObservationType.MEGATAG_2
        //         )
        //     );
        // }
        
        // poseObservationsMT2 = new PoseObservation[poseObservations.size()];
        // for (int i = 0; i < poseObservations.size(); i++)
        //     poseObservationsMT2[i] = poseObservations.get(i);
    }


    // ========================================================
    //                 Pose/Translation Getters
    // ========================================================
        
    private Optional<PoseEstimate> getPoseEstimateMT1() {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        return poseEstimate == null ? Optional.empty() : Optional.of(poseEstimate);
    }

    public Optional<Pose2d> getEstimatedPoseMT1() {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        return poseEstimate == null ? Optional.empty() : Optional.of(poseEstimate.pose);
    }

    // public PoseEstimate getEstimatedPoseMT2() {
    // public Optional<PoseObservation> getEstimatedPoseMT2() {
    //     // LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    //     // return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    //     if (poseObservationsMT2 == null || poseObservationsMT2.length == 0)
    //         return Optional.empty();
    //     else
    //         return Optional.of(poseObservationsMT2[0]);
    // }

    // public Optional<PoseObservation> getEstimatedPoseMT2() {
    //     if (poseObservations.length != 0)
    //         return Optional.of(poseObservations[0]
    //     else
    //         return Optional.empty();
    // }
    
    public void fuseEstimatedPose(SwerveDrivePoseEstimator odometry) {
        if (!hasTarget())
            return;
        
        Optional<PoseEstimate> estimatedPoseEstimate = getPoseEstimateMT1();
        if (!estimatedPoseEstimate.isPresent())
            return;
        Pose2d estimatedPose = estimatedPoseEstimate.get().pose;
        // if (Math.abs(estimatedPose.getZ()) > 0.4)
        //     return;

        int numTagsSeen = getNumberOfTagsSeen();
        double distance = getDistanceEstimatedPose();

        if (numTagsSeen == 1 && distance > 1.3)
            return; 
        
        // this may not even be necessary
        if (numTagsSeen >= 2 && distance > 3.3)
            return;    
            
        Pose2d odoCurrent = odometry.getEstimatedPosition();

        double distX = estimatedPose.getX() - odoCurrent.getX();
        double distY = estimatedPose.getY() - odoCurrent.getY();
        if (Math.sqrt((distX * distX) + (distY * distY)) > 3)
            return;

        double deviation;
        if (numTagsSeen == 1)
            deviation = Constants.AutoAlign.k1TagStdDevs.get(distance);
        else
            deviation = Constants.AutoAlign.k2TagStdDevs.get(distance);
        
        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(
            deviation, deviation, 30
        ));

        odometry.addVisionMeasurement(estimatedPose, estimatedPoseEstimate.get().timestampSeconds);
    }

    // =======================================================
    //                 T-Something Raw Getters
    // =======================================================
    
    public double getTx() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTy() {
        return LimelightHelpers.getTY(limelightName);
    }

    // ================================================
    //                 Distance Getters
    // ================================================

    // distance to CENTER OF ROBOT
    public double getDistanceEstimatedPose() {
        if (!hasTarget())
            return 0;

        var aprilTagPose = aprilTagFieldLayout.getTagPose(getTargetID());
        if (!aprilTagPose.isPresent())
            return 0;

        Pose2d tag = aprilTagPose.get().toPose2d();

        Optional<PoseEstimate> estimatedPoseEstimate = getPoseEstimateMT1();
        if (!estimatedPoseEstimate.isPresent())
            return 0;
        Pose2d robotPose = estimatedPoseEstimate.get().pose;

        double dx = tag.getX() - robotPose.getX();
        double dy = tag.getY() - robotPose.getY();
        
        return Math.sqrt(dx * dx + dy * dy);
    }

    // distance to CAMERA LENS
    public double getDistanceTy() {
        // TODO: is the logic correct?
        if (!hasTarget())
            return 0;

        int tagNum = getTargetID();
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(tagNum);
        if (!tagPose.isPresent())
            return 0;
        
        double tagHeight = tagPose.get().getZ();
        return calculateDistanceToTargetMeters(
            cameraUpOffset, tagHeight, cameraPitchRadians, Math.toRadians(getTy())
        );
    }

    public double getFilteredDistanceEstimatedPose(){
        return distEstimatedPoseFilter.lastValue();
    }

    public double getFilteredDistanceTy(){
        return distTyFilter.lastValue();
    }

    // ======================================================
    //                 Other AprilTag Getters
    // ======================================================
    
    // TODO: test
    public int getNumberOfTagsSeen() {
        return LimelightHelpers.getTargetCount(limelightName);
    }

    public int getTargetID() {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public static Pose2d getAprilTagPose(int tag) {
        var aprilTagPose = aprilTagFieldLayout.getTagPose(tag);
        if (!aprilTagPose.isPresent())
            return new Pose2d();
        return aprilTagPose.get().toPose2d();
    }

    public Pose2d getAprilTagPose() {
        var aprilTagPose = aprilTagFieldLayout.getTagPose(getTargetID());
        if (!aprilTagPose.isPresent())
            return new Pose2d();
        return aprilTagPose.get().toPose2d();
    }

    // ====================================================
    //                 Pipeline Controllers
    // ====================================================

    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

    // ===============================================
    //                 Average Getters
    // ===============================================
    
    public double getTxAverage() {
        return txAverage.getAverage();
    }

    public double getTyAverage() {
        return tyAverage.getAverage();
    }

    // ===========================================================
    //                 Rolling Average Controllers
    // ===========================================================
    
    public void updateRollingAverages() {
        if (!hasTarget())
            return;

        txAverage.add(getTx());
        tyAverage.add(getTy());

        double distTy = getDistanceTy();
        if (distTy != 0)
            distTyFilter.calculate(distTy);

        double distEstimatedPose = getDistanceEstimatedPose();
        if (distEstimatedPose != 0)
            distEstimatedPoseFilter.calculate(distEstimatedPose);
    }

    public void resetRollingAverages(){
        txAverage.clear();
        tyAverage.clear();
    }

    // ======================================
    //                 Others
   // ======================================

    public double getTotalLatencyInMS() {
        return LimelightHelpers.getLatency_Capture(limelightName) + LimelightHelpers.getLatency_Pipeline(limelightName);
    }
    
    public String getName() {
        return limelightName;
    }
    
    // TODO
    public void setBlinking(boolean blinking) {
        // LimelightHelpers.
        // camera.setLED(blinking ? VisionLEDMode.kBlink : VisionLEDMode.kOff);
    }
    
    // private static Pose3d parsePose(double[] rawLLArray) {
    //     return new Pose3d(
    //         rawLLArray[0],
    //         rawLLArray[1],
    //         rawLLArray[2],
    //         new Rotation3d(
    //             Units.degreesToRadians(rawLLArray[3]),
    //             Units.degreesToRadians(rawLLArray[4]),
    //             Units.degreesToRadians(rawLLArray[5])
    //         )
    //     );
    // }
}
