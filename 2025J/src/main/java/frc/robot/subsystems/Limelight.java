package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import frc.robot.utils.LiveData;
import frc.robot.utils.RollingAverage;

public abstract class Limelight extends SubsystemBase {
    private static AprilTagFieldLayout aprilTagFieldLayout;

    private RollingAverage txAverage, tyAverage;
    private LinearFilter distTyFilter, distEstimatedPoseFilter;

    private String limelightName;

    private boolean isInverted;

    public double initialFlashTime;

    private Field2d fieldMT2;

    private int lastSeenTag;

    // private StructPublisher<Pose2d> publisherMT2;

    private LiveData tx, ty, poseDistance, tagsSeen, fieldData, latency, filteredDistanceEstimatedPose,
            filteredDistanceTY, targetID, hasTargetData, currentPriority;

    protected Limelight(String limelightName, boolean isInverted) {
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        this.limelightName = limelightName;
        this.isInverted = isInverted;

        // txAverage = LinearFilter.singlePoleIIR(0.24, 0.02);
        txAverage = new RollingAverage();
        tyAverage = new RollingAverage();

        distTyFilter = LinearFilter.singlePoleIIR(0.24, 0.02);
        distEstimatedPoseFilter = LinearFilter.singlePoleIIR(0.24, 0.02);

        fieldMT2 = new Field2d();
        SmartDashboard.putData(limelightName + " estimated pose (MT2)", fieldMT2);

        initialFlashTime = 0.0;

        // publisherMT2 = NetworkTableInstance.getDefault().getStructTopic(
        //         limelightName + " estimated pose for advantagescope (MT2)", Pose2d.struct).publish();

        // ELASTIC SETUP -- AWAITING CONFIGURATION
        fieldData = new LiveData(fieldMT2, limelightName + " Estimated Pose");
        tx = new LiveData(txAverage.getAverage(), limelightName + " tx");
        ty = new LiveData(tyAverage.getAverage(), limelightName + " ty");
        poseDistance = new LiveData(getDistanceEstimatedPose(), limelightName + " Pose Estimated Distance(To Center)");
        tagsSeen = new LiveData(getNumberOfTagsSeen(), limelightName + " Number of Tags Seen");
        latency = new LiveData(getTotalLatencyInMS(), limelightName + " Latency(MS)");
        filteredDistanceEstimatedPose = new LiveData(getFilteredDistanceEstimatedPose(),
                limelightName + " Filtered Distance Estimated Pose");
        filteredDistanceTY = new LiveData(getFilteredDistanceTy(), limelightName + " Filtered Distance TY");
        targetID = new LiveData(getTargetID(), limelightName + " Target ID");
        hasTargetData = new LiveData(hasTarget(), limelightName + " Has Target");
        currentPriority = new LiveData(LimelightHelpers.getLimelightNTDouble(limelightName, "priorityid"),
                limelightName + " Current Priority");
        lastSeenTag = 0;
    }

    public void setPriorityTag(int tagNum) {
        LimelightHelpers.setPriorityTagID(limelightName, tagNum);
    }

    private void findLateralDepthErrors() {
        Pose2d tagPose = Limelight.getAprilTagPose(getTargetID());
        double tagAngle = tagPose.getRotation().getRadians();

        Optional<Pose2d> estimatedPose = getEstimatedPoseMT2();
        if (estimatedPose.isEmpty())
            return;

        double xError = estimatedPose.get().getX() - tagPose.getX();
        double yError = estimatedPose.get().getY() - tagPose.getY();

        double depthError = xError * Math.cos(tagAngle) + yError * Math.sin(tagAngle);
        double lateralError = xError * Math.sin(tagAngle) - yError * Math.cos(tagAngle);

        SmartDashboard.putNumber(limelightName + " depthError", depthError);
        SmartDashboard.putNumber(limelightName + " lateralError", lateralError);
    }

    @Override
    public void periodic() {
        updateRollingAverages();
        
        double currentTime = Timer.getFPGATimestamp();
        if(currentTime-initialFlashTime > 1.0){
            setLED(LightMode.OFF);
        }

        // limelight wants robot orientation in blue side degrees
        double gyro = Drivetrain.getInstance().getHeadingBlue();
        LimelightHelpers.SetRobotOrientation(
            limelightName, gyro,
            0, 0, 0, 0, 0
        );

        Optional<Pose2d> estimatedPoseMT2 = getEstimatedPoseMT2();
        if (estimatedPoseMT2.isPresent()) {
            fieldMT2.setRobotPose(estimatedPoseMT2.get());
            // publisherMT2.set(estimatedPoseMT2.get());
        }

        if (hasTarget())
            lastSeenTag = getTargetID();    

        fieldData.setData(fieldMT2);
        tx.setNumber(txAverage.getAverage());
        ty.setNumber(tyAverage.getAverage());
        poseDistance.setNumber(getDistanceEstimatedPose());
        tagsSeen.setNumber(getNumberOfTagsSeen());
        latency.setNumber(getTotalLatencyInMS());
        filteredDistanceEstimatedPose.setNumber(getFilteredDistanceEstimatedPose());
        filteredDistanceTY.setNumber(getFilteredDistanceTy());
        targetID.setNumber(getTargetID());
        hasTargetData.setBoolean(hasTarget());
        currentPriority.setNumber(LimelightHelpers.getLimelightNTDouble(limelightName, "priorityid"));

        SmartDashboard.putNumber(limelightName + " tx", getTx());
        SmartDashboard.putNumber(limelightName + " tx average", getTxAverage());

        // TODO: comment out for real match
        findLateralDepthErrors();
    }

    // ========================================================
    //                 Pose/Translation Getters
    // ========================================================

    // private Optional<PoseEstimate> getPoseEstimateMT1() {
    //     PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    //     return poseEstimate == null ? Optional.empty() : Optional.of(poseEstimate);
    // }

    // public Optional<Pose2d> getEstimatedPoseMT1() {
    //     PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    //     return poseEstimate == null ? Optional.empty() : Optional.of(poseEstimate.pose);
    // }

    private Optional<PoseEstimate> getPoseEstimateMT2() {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        return poseEstimate == null ? Optional.empty() : Optional.of(poseEstimate);
    }

    public Optional<Pose2d> getEstimatedPoseMT2() {
        PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        return poseEstimate == null ? Optional.empty() : Optional.of(poseEstimate.pose);
    }

    public void fuseEstimatedPose(SwerveDrivePoseEstimator odometry) {
        if (!hasTarget())
            return;

        Optional<PoseEstimate> estimatedPoseEstimate = getPoseEstimateMT2();
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
    //                 T-Something Raw Getters
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

        Optional<PoseEstimate> estimatedPoseEstimate = getPoseEstimateMT2();
        if (!estimatedPoseEstimate.isPresent())
            return 0;
        Pose2d robotPose = estimatedPoseEstimate.get().pose;

        double dx = tag.getX() - robotPose.getX();
        double dy = tag.getY() - robotPose.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }

    public double getFilteredDistanceEstimatedPose() {
        return distEstimatedPoseFilter.lastValue();
    }

    public double getFilteredDistanceTy() {
        return distTyFilter.lastValue();
    }

    // ======================================================
    //                 Other AprilTag Getters
    // ======================================================

    public int getNumberOfTagsSeen() {
        return LimelightHelpers.getTargetCount(limelightName);
    }

    public int getTargetID() {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public int getLastSeenTag() {
        return lastSeenTag;
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

        double distEstimatedPose = getDistanceEstimatedPose();
        if (distEstimatedPose != 0)
            distEstimatedPoseFilter.calculate(distEstimatedPose);
    }

    public void resetRollingAverages() {
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
    
    public enum LightMode {
        PIPELINE,
        OFF,
        ON,
        BLINK
    }

    public void setLED(LightMode mode) {
        switch (mode) {
            case PIPELINE:
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
                break;
            case OFF:
                LimelightHelpers.setLEDMode_ForceOff(limelightName);
                break;
            case ON:
                LimelightHelpers.setLEDMode_ForceOn(limelightName);
                break;
            case BLINK:
                LimelightHelpers.setLEDMode_ForceBlink(limelightName);
                break;
        }
    }

    public void flashLED(){
        initialFlashTime = Timer.getFPGATimestamp();
        setLED(LightMode.BLINK);
    }
}

