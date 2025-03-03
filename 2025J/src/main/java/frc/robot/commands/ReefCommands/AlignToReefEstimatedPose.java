// // TODO: rectify brokenness

// package frc.robot.commands.ReefCommands;

// import java.util.Optional;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.LimelightFrontLeft;
// import frc.robot.subsystems.LimelightFrontMiddle;
// import frc.robot.subsystems.LimelightFrontRight;
// import frc.robot.subsystems.Limelight;
// import frc.robot.utils.CalculateReefTarget;
// import frc.robot.utils.Constants.AlignmentConstants;
// import frc.robot.utils.Constants.AlignmentConstants.AlignmentDestination;
// import frc.robot.utils.Constants.AlignmentConstants.ReefAlignEstimatedPose;

// public class AlignToReefEstimatedPose extends Command {
//     private Drivetrain drivetrain;
//     private Limelight[] cameras;

//     private PIDController translatePIDController, rotationPIDController;

//     private double translateP, translateI, translateD, translateFF, translateThreshold;
//     private double rotationP, rotationI, rotationD, rotationFF, rotationThreshold;
//     private double rotationLowerP, rotationUseLowerPThreshold;
//     private double maxSpeed;

//     private Optional<Pose2d> desiredPose;
//     private double desiredAngle;

//     private String commandName;
//     private double tagBackMagnitude, tagLateralMagnitude;

//     public AlignToReefEstimatedPose(AlignmentDestination destination) {
//         drivetrain = Drivetrain.getInstance();
        
//         // center
//         switch (destination) {
//             case LEFT -> {
//                 cameras = new Limelight[] {
//                     LimelightFrontRight.getInstance(),
//                     LimelightFrontMiddle.getInstance(),
//                     LimelightFrontLeft.getInstance(),
//                 };
//                 commandName = "left align";
//                 tagLateralMagnitude = AlignmentConstants.ReefAlignEstimatedPose.kLeftOffset;
//             }
//             case MIDDLE -> {
//                 cameras = new Limelight[] {
//                     LimelightFrontMiddle.getInstance(),
//                     LimelightFrontLeft.getInstance(),
//                     LimelightFrontRight.getInstance(),
//                 };
//                 commandName = "middle align";
//                 tagLateralMagnitude = AlignmentConstants.ReefAlignEstimatedPose.kMiddleOffset;
//             }
//             case RIGHT -> {
//                 cameras = new Limelight[] {
//                     LimelightFrontLeft.getInstance(),
//                     LimelightFrontMiddle.getInstance(),
//                     LimelightFrontRight.getInstance(),
//                 };
//                 commandName = "right align";
//                 tagLateralMagnitude = AlignmentConstants.ReefAlignEstimatedPose.kRightOffset;
//             }
//         }

//         translateP = ReefAlignEstimatedPose.kTranslateP;
//         translateI = ReefAlignEstimatedPose.kTranslateI;
//         translateD = ReefAlignEstimatedPose.kTranslateD;
//         translateFF = ReefAlignEstimatedPose.kTranslateFF;
//         translateThreshold = ReefAlignEstimatedPose.kTranslateThreshold;

//         rotationP = ReefAlignEstimatedPose.kRotationP;
//         rotationI = ReefAlignEstimatedPose.kRotationI;
//         rotationD = ReefAlignEstimatedPose.kRotationD;
//         rotationFF = ReefAlignEstimatedPose.kRotationFF;
//         rotationThreshold = ReefAlignEstimatedPose.kRotationThreshold;
//         rotationLowerP = ReefAlignEstimatedPose.kRotationLowerP;
//         rotationUseLowerPThreshold = ReefAlignEstimatedPose.kRotationUseLowerPThreshold;
        
//         translatePIDController = new PIDController(translateP, translateI, translateD);
//         rotationPIDController = new PIDController(rotationP, rotationI, rotationD);
        
//         tagBackMagnitude = ReefAlignEstimatedPose.kTagBackMagnitude;
        
//         maxSpeed = ReefAlignEstimatedPose.kMaxSpeed;

//         SmartDashboard.putNumber(commandName + " lateral offset", tagLateralMagnitude);
//         SmartDashboard.putNumber(commandName + " back offset", tagBackMagnitude);

//         addRequirements(drivetrain);
        
//         {
//             SmartDashboard.putNumber("align translateP", translateP);
//             SmartDashboard.putNumber("align translateI", translateI);
//             SmartDashboard.putNumber("align translateD", translateD);
//             SmartDashboard.putNumber("align translateFF", translateFF);
//             SmartDashboard.putNumber("align translateThreshold", translateThreshold);
            
//             SmartDashboard.putNumber("align rotationP", rotationP);
//             SmartDashboard.putNumber("align rotationI", rotationI);
//             SmartDashboard.putNumber("align rotationD", rotationD);
//             SmartDashboard.putNumber("align rotationFF", rotationFF);
//             SmartDashboard.putNumber("align rotationThreshold", rotationThreshold);
//             SmartDashboard.putNumber("align rotationLowerP", rotationLowerP);
//             SmartDashboard.putNumber("align rotationUseLowerPThreshold", rotationUseLowerPThreshold);

//             SmartDashboard.putNumber("align maxSpeed", maxSpeed);
//         }
//     }

//     @Override
//     public void initialize() {
//         // if aligning from unacceptable position ("sad face case"): returns 0
//         int desiredTarget = CalculateReefTarget.calculateTargetID();
//         SmartDashboard.putNumber("Align Desired Target", desiredTarget);

//         if (!AlignmentConstants.kReefDesiredAngle.containsKey(desiredTarget)) {
//             desiredPose = Optional.empty();
//             return;
//         }
//         desiredAngle = AlignmentConstants.kReefDesiredAngle.get(desiredTarget);
        
//         Pose2d tagPose = Limelight.getAprilTagPose(desiredTarget);
//         double tagAngle = tagPose.getRotation().getRadians();

//         tagLateralMagnitude = SmartDashboard.getNumber(commandName + " lateral offset", tagLateralMagnitude);
//         tagBackMagnitude = SmartDashboard.getNumber(commandName + " back offset", tagBackMagnitude);

//         desiredPose = Optional.of(new Pose2d(
//             tagPose.getX() + tagBackMagnitude * Math.cos(tagAngle) + tagLateralMagnitude * Math.sin(tagAngle),
//             tagPose.getY() + tagBackMagnitude * Math.sin(tagAngle) - tagLateralMagnitude * Math.cos(tagAngle),
//             new Rotation2d(0)
//         ));
//     }
    
//     private Optional<Pose2d> getBestEstimatedPose() {
//         for (Limelight camera : cameras) {
//             Optional<Pose2d> measurement = camera.getEstimatedPoseMT2();
//             if (measurement.isPresent() && camera.hasTarget())
//                 return Optional.of(measurement.get());
//         }

//         return Optional.empty();
//     }

//     @Override
//     public void execute() {
//         {
//             translateP = SmartDashboard.getNumber("align translateP", translateP);
//             translateI = SmartDashboard.getNumber("align translateI", translateI);
//             translateD = SmartDashboard.getNumber("align translateD", translateD);
//             translateFF = SmartDashboard.getNumber("align translateFF", translateFF);
//             translateThreshold = SmartDashboard.getNumber("align translateThreshold", translateThreshold);

//             rotationP = SmartDashboard.getNumber("align rotationP", rotationP);
//             rotationI = SmartDashboard.getNumber("align rotationI", rotationI);
//             rotationD = SmartDashboard.getNumber("align rotationD", rotationD);
//             rotationFF = SmartDashboard.getNumber("align rotationFF", rotationFF);
//             rotationThreshold = SmartDashboard.getNumber("align rotationThreshold", rotationThreshold);
//             rotationLowerP = SmartDashboard.getNumber("align rotationLowerP", rotationLowerP);
//             rotationUseLowerPThreshold = SmartDashboard.getNumber("align rotationUseLowerPThreshold", rotationUseLowerPThreshold);

//             maxSpeed = SmartDashboard.getNumber("align maxSpeed", maxSpeed);
//         }
        
//         if (desiredPose.isEmpty())
//             return;

//         double rotationError = drivetrain.getHeading() - desiredAngle;

//         translatePIDController.setPID(translateP, translateI, translateD);
//         if(Math.abs(rotationError) < rotationUseLowerPThreshold)
//             rotationPIDController.setP(rotationLowerP);
//         else
//             rotationPIDController.setP(rotationP);
//         rotationPIDController.setI(rotationI);
//         rotationPIDController.setD(rotationD);

//         double rotation = 0;
//         if (Math.abs(rotationError) > rotationThreshold)
//           rotation = rotationPIDController.calculate(rotationError) + Math.signum(rotationError) * rotationFF;

//         Optional<Pose2d> estimatedPoseOptional = getBestEstimatedPose();
//         if (!estimatedPoseOptional.isPresent()) {
//             drivetrain.drive(new Translation2d(0, 0), 0, true, null);
//             return;
//         }
//         Pose2d estimatedPose = estimatedPoseOptional.get();

//         double xError = estimatedPose.getX() - desiredPose.get().getX();
//         double yError = estimatedPose.getY() - desiredPose.get().getY();

//         SmartDashboard.putNumber("align xError", xError);
//         SmartDashboard.putNumber("align yError", yError);
        
//         double xTranslate = 0, yTranslate = 0;
//         if (Math.abs(xError) > translateThreshold)
//             xTranslate = translatePIDController.calculate(xError) + Math.signum(xError) * translateFF;
//         if (Math.abs(yError) > translateThreshold)
//             yTranslate = translatePIDController.calculate(yError) + Math.signum(yError) * translateFF;

//         Translation2d translation = new Translation2d(xTranslate, yTranslate);
//         double translateX = translation.getX();
//         double translateY = translation.getY();
//         double translateX_sgn = Math.signum(translateX);
//         double translateY_sgn = Math.signum(translateY);
//         double desaturatedX = Math.min(Math.abs(translateX), maxSpeed);
//         double desaturatedY = Math.min(Math.abs(translateY), maxSpeed);
//         translation = new Translation2d(translateX_sgn * desaturatedX, translateY_sgn * desaturatedY);

//         drivetrain.drive(translation, rotation, true, null);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         if (desiredPose.isPresent())
//             drivetrain.drive(new Translation2d(0,0), 0, false, null);
//     }

//     @Override
//     public boolean isFinished() {
//         return desiredPose.isEmpty();
//     }
// }
