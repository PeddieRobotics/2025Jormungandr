// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.LiveData;
import frc.robot.utils.RobotMap;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain instance;

    private final SwerveModule[] swerveModules;
    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

    private SwerveModuleState[] swerveModuleStates;
    private SwerveModulePosition[] swerveModulePositions;
    private SwerveDrivePoseEstimator odometry, pureOdometry;

    private double currentDrivetrainSpeed = 0;

    private final Pigeon2 gyro;
    private double heading, autoAdjustHeading;

    private final Field2d fusedOdometry;
    private final StructPublisher<Pose2d> fusedOdometryAdvScope, pureOdometryAdvScope;
    private SendableChooser<String> autoStartPosition;
    
    private Translation2d currentMovement;

    private static int pipelineNumber;

    private LiveData odometryX, odometryY, headingData, fusedOdometryData; 

    public Drivetrain() {
        frontLeftModule = new SwerveModule(RobotMap.CANIVORE_BUS, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
                RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID,
                DriveConstants.kFrontLeftMagnetOffset);
        frontRightModule = new SwerveModule(RobotMap.CANIVORE_BUS, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
                RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID,
                DriveConstants.kFrontRightMagnetOffset);
        backLeftModule = new SwerveModule(RobotMap.CANIVORE_BUS, RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
                RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID,
                DriveConstants.kBackLeftMagnetOffset);
        backRightModule = new SwerveModule(RobotMap.CANIVORE_BUS, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
                RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID,
                DriveConstants.kBackRightMagnetOffset);

        swerveModules = new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
        swerveModulePositions = new SwerveModulePosition[] { frontLeftModule.getPosition(), frontRightModule.getPosition(),
                backLeftModule.getPosition(), backRightModule.getPosition() };
        swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

        gyro = new Pigeon2(RobotMap.GYRO_ID, RobotMap.CANIVORE_BUS);
        gyro.setYaw(0);
        autoAdjustHeading = 0.0;

        odometry = new SwerveDrivePoseEstimator(DriveConstants.kKinematics, getHeadingAsRotation2d(), swerveModulePositions, new Pose2d());
        pureOdometry = new SwerveDrivePoseEstimator(DriveConstants.kKinematics, getHeadingAsRotation2d(), swerveModulePositions, new Pose2d());
        pipelineNumber = 0;

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> frontLeftModule.getAngle(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> frontLeftModule.getVelocity(), null);

                builder.addDoubleProperty("Front Right Angle", () -> frontRightModule.getAngle(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> frontRightModule.getVelocity(), null);

                builder.addDoubleProperty("Back Left Angle", () -> backLeftModule.getAngle(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> backLeftModule.getVelocity(), null);

                builder.addDoubleProperty("Back Right Angle", () -> backRightModule.getAngle(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> backRightModule.getVelocity(), null);

                builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getRadians(), null);
            }
        });
        
        currentMovement = new Translation2d();

        fusedOdometry = new Field2d();
        SmartDashboard.putData("Fused odometry", fusedOdometry);
        fusedOdometryAdvScope = NetworkTableInstance.getDefault().getStructTopic("fused odometry for advantagescope", Pose2d.struct).publish();
        pureOdometryAdvScope = NetworkTableInstance.getDefault().getStructTopic("nonfused odometry for advantagescope", Pose2d.struct).publish();

        odometryX = new LiveData(getPose().getX(), "Odometry X");
        odometryY = new LiveData(getPose().getY(), "Odometry Y");
        headingData = new LiveData(getHeading(), "Gyro Heading");
        fusedOdometryData = new LiveData(fusedOdometry, "Fused odometry");

        configureAutoSetupSelector();
        
    }

    /**
     * returns the existing drivetrain instance or creates it if it doesn't exist
     */
    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }

    public void configureAutoSetupSelector(){
        autoStartPosition = new SendableChooser<>();
        autoStartPosition.setDefaultOption("NONE/TELEOP", "NONE/TELEOP");
        autoStartPosition.addOption("LEFT", "LEFT");
        autoStartPosition.addOption("RIGHT", "RIGHT");
        autoStartPosition.addOption("CENTER", "CENTER");

        SmartDashboard.putData("Auto Starting Direction", autoStartPosition);
    }

    public double getAutoAdjustHeading(){
        if(autoStartPosition.getSelected().equals("NONE/TELEOP")){
            return 0;
        } else if (autoStartPosition.getSelected().equals("LEFT")){
            return -90.0;
        } else if (autoStartPosition.getSelected().equals("RIGHT")){
            return 90.0;
        } else if (autoStartPosition.getSelected().equals("CENTER")){
            return 180.0;
        } else {
            return 0;
        }
    }

    /**
     * @return pipeline number for limelight
     */
    public static int getPipelineNumber(){
        return pipelineNumber;
    }

    /**
     * commands the robot to drive
     * 
     * @param translation - translation input (x,y meters/sec in field space)
     * @param rotation - rotation input (degrees/sec)
     * @param fieldOriented - whether the robot is field oriented (true) or robot oriented (false)
     * @param centerOfRotation - robot's center of rotation
     */
    public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRotation) {
        if (fieldOriented)
            currentMovement = translation;

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        ChassisSpeeds robotRelativeSpeeds;
        if (fieldOriented) {
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingAsRotation2d());
        } else {
            robotRelativeSpeeds = fieldRelativeSpeeds;
        }

        currentDrivetrainSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2)
                                + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

        swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxModuleSpeed);
        optimizeModuleStates();
        setSwerveModuleStates(swerveModuleStates);
    }


    public void driveBlue(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRotation) {
        if (fieldOriented)
            currentMovement = translation;

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        ChassisSpeeds robotRelativeSpeeds;
        if (fieldOriented) {
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, new Rotation2d(Math.toRadians(getHeadingBlue())));
        } else {
            robotRelativeSpeeds = fieldRelativeSpeeds;
        }

        currentDrivetrainSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2)
                                + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

        swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxModuleSpeed);
        optimizeModuleStates();
        setSwerveModuleStates(swerveModuleStates);
    }

    /**
     * PLEASE ONLY USE THIS IN AUTONOMOUS
     * commands the robot to drive with the auto angle adjustment
     * 
     * @param translation - translation input (x,y meters/sec in field space)
     * @param rotation - rotation input (degrees/sec)
     * @param fieldOriented - whether the robot is field oriented (true) or robot oriented (false)
     * @param centerOfRotation - robot's center of rotation
     */
    public void driveBlueForceAdjust(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRotation) {
        if (fieldOriented)
            currentMovement = translation;

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        ChassisSpeeds robotRelativeSpeeds;
        if (fieldOriented) {
            Rotation2d rotation2d = new Rotation2d(Math.toRadians(getHeadingBlueForceAdjust()));
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, rotation2d);
        } else {
            robotRelativeSpeeds = fieldRelativeSpeeds;
        }

        currentDrivetrainSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2)
                                + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

        swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxModuleSpeed);
        optimizeModuleStates();
        setSwerveModuleStates(swerveModuleStates);
    }

    /**
     * Only used during autonomous, sets driving strictly to robot relative
     * 
     * @param robotRelativeSpeeds
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        setSwerveModuleStates(swerveModuleStates);
    }

    /**
     * @return returns swerveModuleStates, which contain speed and angle of a 
     * swerve module that tell you how to get to desired position
     */
    public SwerveModuleState[] getSwerveModuleStates(){
        return swerveModuleStates;
    }

    /**
     * optimizes the angle in each module state, will turn the closer direction
     */
    public void optimizeModuleStates() {
        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModuleStates[i].optimize(new Rotation2d(swerveModules[i].getCANCoderReading()));
        }
    }

    /**
     * gets the module position from each swerveModule
     */
    public void updateModulePositions() {
        for (int i = 0; i < swerveModulePositions.length; i++) {
            swerveModulePositions[i] = swerveModules[i].getPosition();
        }
    }

    /**
     * updates odometry, where the robot thinks it is, using cameras and our current odometry
     */
    public void updateOdometry() {
        if (!DriverStation.isAutonomous()) {
            // LimelightBack.getInstance().fuseEstimatedPose(odometry);
            LimelightFrontLeft.getInstance().fuseEstimatedPose(odometry);
            LimelightFrontMiddle.getInstance().fuseEstimatedPose(odometry);
            LimelightFrontRight.getInstance().fuseEstimatedPose(odometry);
            // LimelightLeft.getInstance().fuseEstimatedPose(odometry);
        }
        
        // if (DriverStation.isAutonomous())
        //     odometry.update(new Rotation2d(Math.toRadians(getHeadingBlueForceAdjust())), swerveModulePositions);
        // else
        odometry.update(new Rotation2d(Math.toRadians(getHeadingBlueNoAdjust())), swerveModulePositions);
    }

    /**
     *
     * @param desiredModuleStates
     */
    public void setSwerveModuleStates(SwerveModuleState[] desiredModuleStates) {
        for (int i = 0; i < desiredModuleStates.length; i++) {
            swerveModules[i].setDesiredState(desiredModuleStates[i]);
        }
    }

    /**
     * @return returns heading in degrees, adjusted for auto start
     */
    public double getHeading() {
        heading = gyro.getYaw().getValueAsDouble() + autoAdjustHeading;
        return Math.IEEEremainder(heading, 360);
    }

    /**
     * @return returns heading in degrees, adjusted for auto start
     */
    public double getHeadingNoAdjust() {
        heading = gyro.getYaw().getValueAsDouble();
        return Math.IEEEremainder(heading, 360);
    }
    
    /**
     * PLEASE ONLY USE THIS IN AUTONOMOUS
     * @return returns heading in degrees, always adjusted to auto start (including in auto)
     */
    public double getHeadingForceAdjust() {
        heading = gyro.getYaw().getValueAsDouble() + getAutoAdjustHeading();
        return Math.IEEEremainder(heading, 360);
    }
    
    /**
     * @return returns the heading in blue-side degrees, adjusted for auto start
     * (0 degrees is ALWAYS facing the red alliance wall, on both alliances)
     */
    public double getHeadingBlue() {
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            return getHeading();
        return Math.IEEEremainder(getHeading() + 180, 360);
    }
    
    /**
     * @return returns the heading in blue-side degrees, adjusted for auto start
     * (0 degrees is ALWAYS facing the red alliance wall, on both alliances)
     */
    public double getHeadingBlueNoAdjust() {
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            return getHeadingNoAdjust();
        return Math.IEEEremainder(getHeadingNoAdjust() + 180, 360);
    }

    /**
     * PLEASE ONLY USE THIS IN AUTONOMOUS
     * @return returns the heading in blue-side degrees, always adjusted to auto start (including in auto)
     * (0 degrees is ALWAYS facing the red alliance wall, on both alliances)
     */
    public double getHeadingBlueForceAdjust() {
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            return getHeadingForceAdjust();
        return Math.IEEEremainder(getHeadingForceAdjust() + 180, 360);
    }

    /**
     * resets gyro
     */
    public void resetGyro() {
        autoAdjustHeading = 0.0;
        gyro.reset();
    }

    /**
     * @return returns gyro heading in terms of Rotation2d, adjusted for auto start
     */
    public Rotation2d getHeadingAsRotation2d() {
        return gyro.getRotation2d().plus(new Rotation2d(Math.toRadians(autoAdjustHeading)));
    }

    /**
     * @return returns rotational velocity in DPS
     */
    public double getRotationalVelocity(){
        return -gyro.getAngularVelocityZWorld().getValueAsDouble();
    }

    /**
     * @return returns X acceleration of gyro
     */
    public double getGyroAccX(){
        return gyro.getAccelerationX().getValueAsDouble();
    }

    /**
     * @return returns Y acceleration of gyro
     */
    public double getGyroAccY(){
        return gyro.getAccelerationY().getValueAsDouble();
    }

    /**
     * @return returns Z acceleration of gyro
     */
    public double getGyroAccZ(){
        return gyro.getAccelerationZ().getValueAsDouble();
    }
    
    public Translation2d getCurrentMovement() {
        return currentMovement;
    }

    /**
     * @return returns current speed
     */
    public double getSpeed() {
        return currentDrivetrainSpeed;
    }

    /**
     * @return returns current pose as a pose2d, which containts both translational and rotational
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Resets gyro and sets robot's position to said pose including both rotation and translation
     * 
     * @param pose - Pose 2D
     */
    public void setPose(Pose2d pose) {
        gyro.reset();
        odometry.resetPosition(new Rotation2d(Math.toRadians(getHeadingBlue())), swerveModulePositions, pose);
    }

    /**
     * Does not reset gyro, only changes where you are on the field
     * 
     * @param translation - Translation 2D
     */
    public void resetTranslation(Translation2d translation) {
        odometry.resetTranslation(translation);
    }
    public void resetPureOdometryTranslation(Translation2d translation) {
        pureOdometry.resetTranslation(translation);
    }

    /**
     * @return returns current robot relative chassis speeds by getting the state of each module and 
     * then converting to chassis speeds
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kKinematics.toChassisSpeeds(
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        );
    }

    public void setAutoAdjustHeading(double angleOffset){
        autoAdjustHeading = angleOffset;
    }


    /**
     * Checks if the robot is skidding. Skid is when modules' translational velocities 
     * differ greatly from each other (e.g. during acceleration, outside interference, etc)
     */
    public boolean isSkidding(){
        //gets rotational velocity of the whole robot
        double currentRotationalVelocity = -getRotationalVelocity()*2*Math.PI/360;


        if (Math.abs(currentRotationalVelocity)<0.05){
            currentRotationalVelocity = 0;
        }

        //gets rotational velocity of each module
        ChassisSpeeds rotationalVelocity = new ChassisSpeeds(0,0, currentRotationalVelocity);
        SwerveModuleState pureRotationalStates[] = DriveConstants.kSkidKinematics.toSwerveModuleStates(rotationalVelocity);
     
        //gets rotational and translational velocity of each module
        SwerveModuleState[] moduleStates = Arrays.copyOf(swerveModuleStates, 4);

        Translation2d[] fullModuleStates = new Translation2d[4];
        Translation2d[] pureTranslationalStates = new Translation2d[4];


        for (int i = 0; i<4; i++){
            fullModuleStates[i] = new Translation2d(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);

            Translation2d pureRotation = new Translation2d(pureRotationalStates[i].speedMetersPerSecond, pureRotationalStates[i].angle);

            //subtracts rotational velocity from full states, leaving only translational velocity
            pureTranslationalStates[i] = fullModuleStates[i].minus(pureRotation);
        }

        //compares the translational velocity of each module to each other, checking for large differences
        for (int i = 0; i<4; i++){
            for (int j = i+1; j<4; j++){
                Translation2d difference = pureTranslationalStates[i].minus(pureTranslationalStates[j]);

                double vtotal = Math.sqrt(Math.pow(difference.getX(),2) + Math.pow(difference.getY(), 2));

                if(vtotal>DriveConstants.kSkidThreshold){
                    return true;
                }

            }
        }

        return false;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("skid", isSkidding());
        updateModulePositions();
        updateOdometry();
        fusedOdometry.setRobotPose(odometry.getEstimatedPosition());
        fusedOdometryAdvScope.set(odometry.getEstimatedPosition());
        pureOdometryAdvScope.set(pureOdometry.getEstimatedPosition());

        for(int i = 0; i < 4; i++){
            SmartDashboard.putNumber("module " + i + "desired speed", swerveModuleStates[i].speedMetersPerSecond);
            SmartDashboard.putNumber("module " + i + "desired angle", swerveModuleStates[i].angle.getRadians());
            SmartDashboard.putNumber("module " + i + "actual speed", swerveModules[i].getVelocity());
            SmartDashboard.putNumber("module " + i + "actual angle", swerveModules[i].getAngle());
        }

        odometryX.setNumber(getPose().getX());   
        odometryY.setNumber(getPose().getY()); 
        headingData.setNumber(getHeading()); 
        fusedOdometryData.setData(fusedOdometry); 
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
