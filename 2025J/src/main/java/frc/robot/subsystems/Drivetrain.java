// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.RobotMap;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  private final SwerveModule[] swerveModules;
  private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;

  private SwerveModuleState[] swerveModuleStates;
  private SwerveModulePosition[] swerveModulePositions;
  private SwerveDrivePoseEstimator odometry;

  private double currentDrivetrainSpeed = 0;

  private final Pigeon2 gyro;
  private double heading;

  private final Field2d field;

  public Drivetrain() {
    frontLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
        RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID,
        DriveConstants.kFrontLeftCancoderOffset);
    frontRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
        RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID,
        DriveConstants.kFrontRightCancoderOffset);
    backLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
        RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID,
        DriveConstants.kBackLeftCancoderOffset);
    backRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
        RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID,
        DriveConstants.kBackRightCancoderOffset);

    swerveModules = new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
    swerveModulePositions = new SwerveModulePosition[] { frontLeftModule.getPosition(), frontRightModule.getPosition(),
        backLeftModule.getPosition(), backRightModule.getPosition() };
    swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    gyro = new Pigeon2(RobotMap.GYRO_ID, RobotMap.CANIVORE_NAME);
    gyro.setYaw(0);

    odometry = new SwerveDrivePoseEstimator(DriveConstants.kinematics, getHeadingAsRotation2d(), swerveModulePositions, new Pose2d());

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

    field = new Field2d();
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

  /**
   * commands the robot to drive
   * 
   * @param translation - translation input (x,y meters/sec in field space)
   * @param rotation - rotation input (degrees/sec)
   * @param fieldOriented - whether the robot is field oriented (true) or robot oriented (false)
   * @param centerOfRotation - robot's center of rotation
   */
  public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerOfRotation) {
    ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    ChassisSpeeds robotRelativeSpeeds;
    if (fieldOriented) {
      robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingAsRotation2d());
    } else {
      robotRelativeSpeeds = fieldRelativeSpeeds;
    }

    currentDrivetrainSpeed = Math.sqrt(Math.pow(robotRelativeSpeeds.vxMetersPerSecond, 2)
                + Math.pow(robotRelativeSpeeds.vyMetersPerSecond, 2));

    swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    // TODO: desaturate later!
    optimizeModuleStates();
    setSwerveModuleStates(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    setSwerveModuleStates(swerveModuleStates);
  }

  public SwerveModuleState[] getSwerveModuleStates(){
    return swerveModuleStates;
  }

  public void optimizeModuleStates() {
    for (int i = 0; i < swerveModuleStates.length; i++) {
      swerveModuleStates[i].optimize(new Rotation2d(swerveModules[i].getCANCoderReading()));
    }
  }

  public void updateModulePositions() {
    for (int i = 0; i < swerveModulePositions.length; i++) {
      swerveModulePositions[i] = swerveModules[i].getPosition();
    }
  }

  public void updateOdometry() {
    odometry.update(getHeadingAsRotation2d(), swerveModulePositions);
  }

  public void setSwerveModuleStates(SwerveModuleState[] desiredModuleStates) {
    for (int i = 0; i < desiredModuleStates.length; i++) {
      swerveModules[i].setDesiredState(desiredModuleStates[i]);
    }
  }

  public double getHeading() {
    heading = -gyro.getYaw().getValueAsDouble();
    return Math.IEEEremainder(heading, 360);
  }

  public void resetGyro() {
    gyro.reset();
  }

  public Rotation2d getHeadingAsRotation2d() {
    return gyro.getRotation2d();
  }

  public double getRotationalVelocity(){
    return -gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public double getGyroAccX(){
    return gyro.getAccelerationX().getValueAsDouble();
  }

  public double getGyroAccY(){
    return gyro.getAccelerationY().getValueAsDouble();
  }

  public double getGyroAccZ(){
    return gyro.getAccelerationZ().getValueAsDouble();
  }

  public double getSpeed() {
    return currentDrivetrainSpeed;
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    gyro.reset();
    odometry.resetPosition(getHeadingAsRotation2d(), swerveModulePositions, pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState());
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
    SwerveModuleState pureRotationalStates[] = DriveConstants.skidKinematics.toSwerveModuleStates(rotationalVelocity);
   
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
    field.setRobotPose(odometry.getEstimatedPosition());
    SmartDashboard.putData("Field", field);

    for(int i = 0; i < 4; i++){
      // SmartDashboard.putNumber("module " + i +"desired speed", swerveModuleStates[i].speedMetersPerSecond);
      // SmartDashboard.putNumber("module " + i +"desired angle", swerveModuleStates[i].angle.getRadians());
      // SmartDashboard.putNumber("module " + i +"actual speed", swerveModules[i].getVelocity());
      // SmartDashboard.putNumber("module " + i +"actual angle", swerveModules[i].getAngle());

    }
    SmartDashboard.putNumber("Odometry X", getPose().getX());
    SmartDashboard.putNumber("Odometry Y", getPose().getY());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
