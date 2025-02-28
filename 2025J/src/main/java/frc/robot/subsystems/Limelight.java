package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LiveData;
import frc.robot.utils.RollingAverage;

public abstract class Limelight extends SubsystemBase {
    /**
     * Algorithm from
     * https://docs.limelightvision.io/en/latest/cs_estimating_distance.html
     * Estimates
     * range to a target using the target's elevation. This method can produce more
     * stable results
     * than SolvePNP when well tuned, if the full 6d robot pose is not required.
     * Note that this method
     * requires the camera to have 0 roll (not be skewed clockwise or CCW relative
     * to the floor), and
     * for there to exist a height differential between goal and camera. The larger
     * this differential,
     * the more accurate the distance estimate will be.
     *
     * <p>
     * Units can be converted using the {@link edu.wpi.first.math.util.Units} class.
     *
     * @param cameraHeightMeters The physical height of the camera off the floor in
     *                           meters.
     * @param targetHeightMeters The physical height of the target off the floor in
     *                           meters. This
     *                           should be the height of whatever is being targeted
     *                           (i.e. if the targeting region is set to
     *                           top, this should be the height of the top of the
     *                           target).
     * @param cameraPitchRadians The pitch of the camera from the horizontal plane
     *                           in radians.
     *                           Positive values up.
     * @param targetPitchRadians The pitch of the target in the camera's lens in
     *                           radians. Positive
     *                           values up.
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

    private static AprilTagFieldLayout aprilTagFieldLayout;

    private RollingAverage txAverage, tyAverage;
    private LinearFilter distTyFilter, distEstimatedPoseFilter;

    private String limelightName;

    private double cameraUpOffset;
    private double cameraPitchRadians;
    private boolean isInverted;

    private Field2d fieldMT1, fieldMT2;
    private StructPublisher<Pose2d> publisherMT1, publisherMT2;

    private LiveData tx, ty, poseDistance, tagsSeen, fieldData, latency, distanceTy, filteredDistanceEstimatedPose,
            filteredDistanceTY, targetID, hasTargetData, currentPriority;

    protected Limelight(String limelightName, double cameraUpOffset, double cameraPitchDegrees, boolean isInverted) {
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        this.limelightName = limelightName;
        this.cameraUpOffset = cameraUpOffset;
        this.cameraPitchRadians = Math.toRadians(cameraPitchDegrees);
        this.isInverted = isInverted;

        txAverage = new RollingAverage();
        tyAverage = new RollingAverage();

        distTyFilter = LinearFilter.singlePoleIIR(0.24, 0.02);
        distEstimatedPoseFilter = LinearFilter.singlePoleIIR(0.24, 0.02);

        fieldMT1 = new Field2d();
        fieldMT2 = new Field2d();
        SmartDashboard.putData(limelightName + " estimated pose (MT1)", fieldMT1);
        SmartDashboard.putData(limelightName + " estimated pose (MT2)", fieldMT2);

        publisherMT1 = NetworkTableInstance.getDefault().getStructTopic(
                limelightName + " estimated pose for advantagescope (MT1)", Pose2d.struct).publish();
        publisherMT2 = NetworkTableInstance.getDefault().getStructTopic(
                limelightName + " estimated pose for advantagescope (MT2)", Pose2d.struct).publish();

        // ELASTIC SETUP -- AWAITING CONFIGURATION
        fieldData = new LiveData(fieldMT2, limelightName + " Estimated Pose");
        tx = new LiveData(txAverage.getAverage(), limelightName + " tx");
        ty = new LiveData(tyAverage.getAverage(), limelightName + " ty");
        poseDistance = new LiveData(getDistanceEstimatedPose(), limelightName + " Pose Estimated Distance(To Center)");
        tagsSeen = new LiveData(getNumberOfTagsSeen(), limelightName + " Number of Tags Seen");
        latency = new LiveData(getTotalLatencyInMS(), limelightName + " Latency(MS)");
        distanceTy = new LiveData(getDistanceTy(), limelightName + " distance (Ty)");
        filteredDistanceEstimatedPose = new LiveData(getFilteredDistanceEstimatedPose(),
                limelightName + " Filtered Distance Estimated Pose");
        filteredDistanceTY = new LiveData(getFilteredDistanceTy(), limelightName + " Filtered Distance TY");
        targetID = new LiveData(getTargetID(), limelightName + " Target ID");
        hasTargetData = new LiveData(hasTarget(), limelightName + " Has Target");
        currentPriority = new LiveData(LimelightHelpers.getLimelightNTDouble(limelightName, "priorityid"),
                limelightName + " Current Priority");
    }

    public void setPriorityTag(int tagNum) {
        LimelightHelpers.setPriorityTagID(limelightName, tagNum);
    }

    @Override
    public void periodic() {
        updateRollingAverages();
        
        // TODO: TEST!!!
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            LimelightHelpers.SetRobotOrientation(
                limelightName,
                Drivetrain.getInstance().getHeading(),
                0, 0, 0, 0, 0
            );
        }
        // RED:
        else {
            LimelightHelpers.SetRobotOrientation(
                limelightName,
                Math.IEEEremainder(Drivetrain.getInstance().getHeading() + 180, 360),
                0, 0, 0, 0, 0
            );
        }

        Optional<Pose2d> estimatedPoseMT1 = getEstimatedPoseMT1();
        if (estimatedPoseMT1.isPresent()) {
            fieldMT1.setRobotPose(estimatedPoseMT1.get());
            publisherMT1.set(estimatedPoseMT1.get());
        }

        Optional<Pose2d> estimatedPoseMT2 = getEstimatedPoseMT2();
        if (estimatedPoseMT2.isPresent()) {
            fieldMT2.setRobotPose(estimatedPoseMT2.get());
            publisherMT2.set(estimatedPoseMT2.get());
        }

        fieldData.setData(fieldMT2);
        tx.setNumber(txAverage.getAverage());
        ty.setNumber(tyAverage.getAverage());
        poseDistance.setNumber(getDistanceEstimatedPose());
        tagsSeen.setNumber(getNumberOfTagsSeen());
        latency.setNumber(getTotalLatencyInMS());
        distanceTy.setNumber(getDistanceTy());
        filteredDistanceEstimatedPose.setNumber(getFilteredDistanceEstimatedPose());
        filteredDistanceTY.setNumber(getFilteredDistanceTy());
        targetID.setNumber(getTargetID());
        hasTargetData.setBoolean(hasTarget());
        currentPriority.setNumber(LimelightHelpers.getLimelightNTDouble(limelightName, "priorityid"));
    }

    // ========================================================
    // Pose/Translation Getters
    // ========================================================

    private Optional<PoseEstimate> getPoseEstimateMT1() {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        return poseEstimate == null ? Optional.empty() : Optional.of(poseEstimate);
    }

    public Optional<Pose2d> getEstimatedPoseMT1() {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        return poseEstimate == null ? Optional.empty() : Optional.of(poseEstimate.pose);
    }

    public Optional<Pose2d> getEstimatedPoseMT2() {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        return poseEstimate == null ? Optional.empty() : Optional.of(poseEstimate.pose);
    }

    public void fuseEstimatedPose(SwerveDrivePoseEstimator odometry) {
        if (!hasTarget())
            return;

        Optional<PoseEstimate> estimatedPoseEstimate = getPoseEstimateMT1();
        if (!estimatedPoseEstimate.isPresent())
            return;
        Pose2d estimatedPose = estimatedPoseEstimate.get().pose;
        // if (Math.abs(estimatedPose.getZ()) > 0.4)
        // return;

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
            deviation = CameraConstants.k1TagStdDevs.get(distance);
        else
            deviation = CameraConstants.k2TagStdDevs.get(distance);

        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(
                deviation, deviation, 30));

        odometry.addVisionMeasurement(estimatedPose, estimatedPoseEstimate.get().timestampSeconds);
    }

    // =======================================================
    // T-Something Raw Getters
    // =======================================================

    // so, uh, Limelight doesn't invert the Tx and Ty automatically on upside down
    // Limelights
    public double getTx() {
        return (isInverted ? -1 : 1) * LimelightHelpers.getTX(limelightName);
    }

    public double getTy() {
        return (isInverted ? -1 : 1) * LimelightHelpers.getTY(limelightName);
    }

    // ================================================
    // Distance Getters
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
                cameraUpOffset, tagHeight, cameraPitchRadians, Math.toRadians(getTy()));
    }

    public double getFilteredDistanceEstimatedPose() {
        return distEstimatedPoseFilter.lastValue();
    }

    public double getFilteredDistanceTy() {
        return distTyFilter.lastValue();
    }

    // ======================================================
    // Other AprilTag Getters
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
    // Pipeline Controllers
    // ====================================================

    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }

    // ===============================================
    // Average Getters
    // ===============================================

    public double getTxAverage() {
        return txAverage.getAverage();
    }

    public double getTyAverage() {
        return tyAverage.getAverage();
    }

    // ===========================================================
    // Rolling Average Controllers
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

    public void resetRollingAverages() {
        txAverage.clear();
        tyAverage.clear();
    }

    // ======================================
    // Others
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
}
