// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants.ScoreConstants;
import frc.robot.utils.LiveData;
import frc.robot.utils.Logger;
import frc.robot.utils.OperatorOI;

import static frc.robot.subsystems.Superstructure.SuperstructureState.*;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;

    private final Arm arm;
    private final Claw claw;
    private final Elevator elevator;
    // private final HPIntake hpIntake;

    private SuperstructureState systemState;
    private SuperstructureState requestedSystemState;

    private Timer timer;

    private LiveData systemStateData, requestedSystemStateData, algaeIndex, coralIndex; 

    private boolean isManualControl, hasJustRemovedAlgae;
    private double startedOuttakingRemovedAlgaeTime;
    
    private boolean autoRemoveAlgaeSwitch;

    // flag used for align
    private ScoringFlag scoringFlag;

    public enum SuperstructureState {
        STOW,
        HP_INTAKE,
        ALGAE_GROUND_INTAKE,
        PRESTAGE,
        L1_PREP,
        L2_PREP,
        L3_PREP,
        L4_PREP,
        L1_SCORE,
        L2_SCORE,
        L3_SCORE,
        L4_SCORE,
        BARGE_PRESTAGE,
        BARGE_PREP,
        BARGE_SCORE,
        PROCESSOR_PREP,
        PROCESSOR_SCORE,
        REEF1_ALGAE_INTAKE,
        REEF2_ALGAE_INTAKE,
        EJECT_ALGAE,
        EJECT_CORAL,
        REMOVING_ALGAE
    }

    public enum ScoringFlag {
        L1FLAG,
        L2FLAG,
        L3FLAG,
        L4FLAG
    }

    public Superstructure() {
        systemState = STOW;
        requestedSystemState = STOW;

        scoringFlag = ScoringFlag.L4FLAG;

        arm = Arm.getInstance();
        claw = Claw.getInstance();
        elevator = Elevator.getInstance();
        // hpIntake = HPIntake.getInstance();

        timer = new Timer();

        algaeIndex = new LiveData(false, "Algae Index"); 
        coralIndex = new LiveData(false, "Coral Index"); 

        systemStateData = new LiveData(systemState.toString(), "System State"); 
        requestedSystemStateData = new LiveData(requestedSystemState.toString(), "Requested System State"); 

        isManualControl = false;
        hasJustRemovedAlgae = false;
        startedOuttakingRemovedAlgaeTime = 0.0;
        
        autoRemoveAlgaeSwitch = false;
    }

    public static Superstructure getInstance() {
        if (superstructure == null) {
            superstructure = new Superstructure();
        }
        return superstructure;
    }

    public void requestState(SuperstructureState request) {
        requestedSystemState = request;
    }

    public SuperstructureState getCurrentState() {
        return systemState;
    }

    public SuperstructureState getRequestedState() {
        return requestedSystemState;
    }

    public void setManualControl(boolean isTrue){
        isManualControl = isTrue;
    }

    public void setL1Flag(){
        scoringFlag = ScoringFlag.L1FLAG;
        SmartDashboard.putBoolean("L1 Flag", true);
        SmartDashboard.putBoolean("L2 Flag", false);
        SmartDashboard.putBoolean("L3 Flag", false);
        SmartDashboard.putBoolean("L4 Flag", false);   
    }

    public void setL2Flag(){
        scoringFlag = ScoringFlag.L2FLAG;
        SmartDashboard.putBoolean("L1 Flag", false);
        SmartDashboard.putBoolean("L2 Flag", true);
        SmartDashboard.putBoolean("L3 Flag", false);
        SmartDashboard.putBoolean("L4 Flag", false); 
    }

    public void setL3Flag(){
        scoringFlag = ScoringFlag.L3FLAG;
        SmartDashboard.putBoolean("L1 Flag", false);
        SmartDashboard.putBoolean("L2 Flag", false);
        SmartDashboard.putBoolean("L3 Flag", true);
        SmartDashboard.putBoolean("L4 Flag", false); 
    }

    public void setL4Flag(){
        scoringFlag = ScoringFlag.L4FLAG;
        SmartDashboard.putBoolean("L1 Flag", false);
        SmartDashboard.putBoolean("L2 Flag", false);
        SmartDashboard.putBoolean("L3 Flag", false);
        SmartDashboard.putBoolean("L4 Flag", true); 
    }

    public ScoringFlag getScoringFlag(){
        return scoringFlag;
    }

    @Override
    public void periodic() {
        double manualOffset = SmartDashboard.getNumber("Scoring Pose Offset", 0);

        systemStateData.setString(systemState.toString()); 
        requestedSystemStateData.setString(requestedSystemState.toString()); 

        algaeIndex.setBoolean(claw.getAlgaeSensor()); 

        switch (systemState) {
            case STOW -> {
                if(!isManualControl){
                    // stop intake
                    // bring elevator 
                    if(elevator.isAtBottom()){
                        elevator.setElevatorNeutralMode();
                    } else{
                        elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorStowPosition);
                    }
                    arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmStowPosition);

                    if (claw.getAlgaeSensor()) {
                        claw.holdAlgae();
                        claw.stopCoralMotor();
                    } else {
                        claw.stopClaw();
                    }
                }

                if (Arrays.asList(
                        STOW, HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        PRESTAGE,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case HP_INTAKE -> {
                // set angle
                // set elevator
                // run intake motor
                if(elevator.isAtBottom()){
                    elevator.setElevatorNeutralMode();
                }else{
                    elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorHPIntakePosition);
                }
                
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmHPIntakePosition);

                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorHPIntakePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmHPIntakePosition);

                // add gate to check elevator height and arm angle ?

                if (hasJustRemovedAlgae) {
                    if (startedOuttakingRemovedAlgaeTime == 0)
                        startedOuttakingRemovedAlgaeTime = Timer.getFPGATimestamp();
                    else if (Timer.getFPGATimestamp() - startedOuttakingRemovedAlgaeTime >= 0.2) {
                        hasJustRemovedAlgae = false;
                        startedOuttakingRemovedAlgaeTime = 0;
                    }
                    claw.intakeCoral(ClawConstants.kCoralIntakeSpeed);
                } else if(claw.bothCoralSensorsTriggered()) {
                    claw.stopClaw();
                    if (DriverStation.isAutonomous())
                        systemState = requestedSystemState;
                    else
                        requestState(PRESTAGE);
                } else if (claw.getTopSensor() && !claw.getBottomSensor()){
                    claw.intakeCoral(ClawConstants.kCoralSlowIntake);
                } else {
                    claw.intakeCoral(ClawConstants.kCoralIntakeSpeed);
                }

                if(claw.getAlgaeSensor()){
                    claw.holdAlgae();
                } else {
                    claw.stopAlgaeMotor();
                }


                if (Arrays.asList(
                        STOW,
                        ALGAE_GROUND_INTAKE,
                        PROCESSOR_PREP,
                        PRESTAGE,
                        EJECT_ALGAE,
                        EJECT_CORAL,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }

                if ((requestedSystemState == L3_PREP || requestedSystemState == L4_PREP) &&
                        DriverStation.isAutonomous() && claw.bothCoralSensorsTriggered()) {
                    systemState = requestedSystemState;
                }
            }

            case ALGAE_GROUND_INTAKE -> {
                // run intake
                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorGroundIntakePosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmGroundIntakePosition);
                
                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorGroundIntakePosition);
                if(elevator.isAtPosition(ScoreConstants.kElevatorGroundIntakePosition)){
                    arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmGroundIntakePosition);
                }

                claw.intakeAlgae();
                claw.stopCoralMotor();

                if (claw.getAlgaeSensor()) {
                    claw.holdAlgae();
                    arm.setArmPositionMotionMagicVoltage(0.17);
                    if(arm.isAtPosition(0.17)){
                        requestState(STOW);
                    }
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        PRESTAGE,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case L1_PREP -> {
                // set prep angle
                // elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorL1ScorePosition);
                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorL1ScorePosition + manualOffset);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmL1ScorePosition);

                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorL1ScorePosition);
                if(elevator.isAtPosition(ScoreConstants.kElevatorL1ScorePosition)){
                    arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmL1ScorePosition);
                }

                claw.stopClaw();

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_SCORE,
                        L2_PREP,
                        L3_PREP,
                        L4_PREP,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case L2_PREP -> {
                // set prep angle
                /*
                    * two different cases:
                    * - dunk case
                    * move to angle close to scoring angle
                    * - shoot case
                    * move to scoring angle
                    */
                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorL2ScorePosition + manualOffset);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmL2ScorePosition);

                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorL2ScorePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmL2ScorePosition);
                claw.stopClaw();

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_SCORE,
                        L3_PREP,
                        L4_PREP,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case L3_PREP -> {
                // set prep angle
                /*
                    * two different cases:
                    * - dunk case
                    * move to angle close to scoring angle
                    * - shoot case
                    * move to scoring angle
                    */
                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorL3ScorePosition + manualOffset);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmL3ScorePosition);
                
                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorL3ScorePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmL3ScorePosition);
                claw.stopClaw();


                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_SCORE,
                        L4_PREP,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case PRESTAGE -> {
                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorPrestagePosition + manualOffset);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmStowPosition);

                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorL4PrestagePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmStowPosition);

                claw.stopCoralMotor();
                if(claw.getAlgaeSensor()){
                    claw.holdAlgae();
                } else {
                    claw.stopClaw();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        PRESTAGE,
                        L4_PREP,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case L4_PREP -> {
                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorL4ScorePosition + manualOffset);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmL4ScorePosition);
                
                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorL4ScorePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmL4ScorePosition);
                claw.stopClaw();


                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_SCORE,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case L1_SCORE -> {
                if (timer.hasElapsed(ScoreConstants.kL1ScoreTimeout) && !claw.eitherCoralSensorTriggered()){
                    timer.reset();
                    claw.stopClaw();
                    arm.setArmPositionVoltage(0.17);
                    if (arm.isAtPosition(0.17)){
                        requestState(HP_INTAKE);
                    }
                } else {
                    // double speed = SmartDashboard.getNumber("Tuning: L1Speed", ClawConstants.kCoralL1OuttakeSpeed);
                    claw.intakeAlgae();
                    claw.outtakeCoralL1();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case L2_SCORE -> {

                if (timer.hasElapsed(ScoreConstants.kL2ScoreTimeout) || !claw.eitherCoralSensorTriggered()){
                    timer.reset();
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                } else {
                    claw.stopAlgaeMotor();
                    claw.outtakeCoral();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case L3_SCORE -> {
                if (timer.hasElapsed(ScoreConstants.kL3ScoreTimeout) || !claw.eitherCoralSensorTriggered()){
                    timer.reset();
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                } else {
                    claw.stopAlgaeMotor();
                    claw.outtakeCoral();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case L4_SCORE -> {
                if (timer.hasElapsed(ScoreConstants.kL4ScoreTimeout) || !claw.eitherCoralSensorTriggered()){
                    timer.reset();
                    if (shouldRemoveAlgae()) {
                        requestState(REMOVING_ALGAE);
                    } else {
                        claw.stopClaw();
                        requestState(HP_INTAKE);
                    }
                } else {
                    claw.stopAlgaeMotor();
                    claw.outtakeCoral();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        REMOVING_ALGAE)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case BARGE_PRESTAGE -> {

                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorBargePrestagePosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmStowPosition);

                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorBargePrestagePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmStowPosition);

                if (claw.getAlgaeSensor()) {
                    claw.holdAlgae();
                    claw.stopCoralMotor();
                } else {
                    claw.stopClaw();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        PRESTAGE,
                        BARGE_PREP,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case BARGE_PREP -> {

                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorBargeScorePosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmBargeScorePosition);

                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorBargeScorePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmBargeScorePosition);

                if (claw.getAlgaeSensor()) {
                    claw.holdAlgae();
                    claw.stopCoralMotor();
                } else {
                    claw.stopClaw();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        PRESTAGE,
                        BARGE_PRESTAGE,
                        BARGE_SCORE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case BARGE_SCORE -> {
                if (timer.hasElapsed(ScoreConstants.kBargeTimeout) && !claw.getAlgaeSensor()){
                    timer.reset();
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                } else {
                    claw.outtakeAlgae();
                    claw.stopCoralMotor();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case PROCESSOR_PREP -> {
                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorProcessorScorePosition);
                //arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmProcessorScorePosition);
                
                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorProcessorScorePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmProcessorScorePosition);

                if (claw.getAlgaeSensor()) {
                    claw.holdAlgae();
                    claw.stopCoralMotor();
                } else {
                    claw.stopClaw();
                }


                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        PRESTAGE,
                        BARGE_PRESTAGE,
                        PROCESSOR_SCORE,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case PROCESSOR_SCORE -> {
                if (timer.hasElapsed(ScoreConstants.kProcessorTimeout) && !claw.getAlgaeSensor()){
                    timer.reset();
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                } else {
                    claw.outtakeAlgae();
                    claw.stopCoralMotor();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case REEF1_ALGAE_INTAKE -> {
                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorReef1IntakePosition + manualOffset);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmReef1IntakePosition);
                
                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorReef1IntakePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmReef1IntakePosition);

                if (claw.getAlgaeSensor()) {
                    claw.holdAlgae();
                    requestState(STOW);
                } else {
                    claw.intakeAlgae();
                    claw.stopCoralMotor();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        PRESTAGE,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case REEF2_ALGAE_INTAKE -> {
                elevator.setElevatorPositionMotionMagicVoltage(ScoreConstants.kElevatorReef2IntakePosition + manualOffset);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmReef2IntakePosition);
                
                //elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorReef2IntakePosition);
                arm.setArmPositionMotionMagicVoltage(ScoreConstants.kArmReef2IntakePosition);
                
                if (claw.getAlgaeSensor()) {
                    claw.holdAlgae();
                    requestState(STOW);
                } else {
                    claw.intakeAlgae();
                    claw.stopCoralMotor();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        PRESTAGE,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case EJECT_ALGAE -> {
                claw.outtakeAlgae();
                claw.stopCoralMotor();
                
                // if (!claw.hasAlgae()) {
                //     claw.stopClaw();
                //     requestState(HP_INTAKE);
                // }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        PRESTAGE,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }

            case EJECT_CORAL -> {
                claw.outtakeCoral();
                claw.stopAlgaeMotor();

                // if (!claw.eitherCoralSensorTriggered()) {
                //     claw.stopClaw();
                //     requestState(HP_INTAKE);
                // }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        PRESTAGE,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }
            case REMOVING_ALGAE -> {
                double armAngle = 0.20;
                if (elevator.getElevatorCANcoderPosition() > 1.5) {
                    claw.intakeAlgae();
                    claw.stopCoralMotor();
                    arm.setArmPositionMotionMagicVoltage(armAngle);
                    elevator.setElevatorPercentOutput(getAlgaeRemovalSpeed());
                }
                else {
                    arm.setArmPositionMotionMagicVoltage(0.25);
                    hasJustRemovedAlgae = true;
                    requestState(STOW);
                }

                if (Arrays.asList(STOW, HP_INTAKE).contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }
        }

    }

    public void sendToScore() {
        double manualOffset = SmartDashboard.getNumber("Scoring Pose Offset", 0);
        
        Logger logger = Logger.getInstance();
        switch (systemState) {
            case L1_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmL1ScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorL1ScorePosition + manualOffset)) {
                    logger.logScoreEvent(1, elevator.getElevatorCANcoderPosition(), arm.getArmMotorEncoderPosition());  
                    requestState(L1_SCORE);
                    timer.reset();
                    timer.start();
                }
            }

            case L2_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmL2ScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorL2ScorePosition + manualOffset)) {
                    logger.logScoreEvent(2, elevator.getElevatorCANcoderPosition(), arm.getArmMotorEncoderPosition());  
                    requestState(L2_SCORE);
                    timer.reset();
                    timer.start();
                }
            }

            case L3_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmL3ScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorL3ScorePosition + manualOffset)) {
                    logger.logScoreEvent(3, elevator.getElevatorCANcoderPosition(), arm.getArmMotorEncoderPosition());  
                    requestState(L3_SCORE);
                    timer.reset();
                    timer.start();
                }
            }

            case L4_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmL4ScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorL4ScorePosition + manualOffset)) {
                    logger.logScoreEvent(4, elevator.getElevatorCANcoderPosition(), arm.getArmMotorEncoderPosition());  
                    requestState(L4_SCORE);
                    timer.reset();
                    timer.start();
                }
            }

            case PROCESSOR_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmProcessorScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorProcessorScorePosition + manualOffset)) {
                    requestState(PROCESSOR_SCORE);
                    timer.reset();
                    timer.start();
                }
            }

            case BARGE_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmBargeScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorBargeScorePosition + manualOffset)) {
                    requestState(BARGE_SCORE);
                    timer.reset();
                    timer.start();
                }
            }
            default -> {
                break;
            }
        }
    }
    
    public void setAutoRemoveAlgaeSwitch(boolean value) {
        autoRemoveAlgaeSwitch = value;
    }
    
    private boolean shouldRemoveAlgae() {
        if (DriverStation.isAutonomous()) {
            boolean value = autoRemoveAlgaeSwitch;
            autoRemoveAlgaeSwitch = false;
            return value;
        }
        // return OperatorOI.getInstance().getLeftBumperHeld();
        // for now
        return false;
    }

    private final List<Integer> highAlgaeTags = Arrays.asList(
        18, 20, 22, 7, 9, 11
    );
    public Optional<Boolean> isHighAlgae() {
        // return SmartDashboard.getBoolean("RemoveAlgae: high?", false);
        Limelight camera = LimelightFrontLeft.getInstance();
        if (camera.getNumberOfTagsSeen() == 1)
            return Optional.of(highAlgaeTags.contains(camera.getTargetID()));

        camera = LimelightFrontRight.getInstance();
        if (camera.getNumberOfTagsSeen() == 1)
            return Optional.of(highAlgaeTags.contains(camera.getTargetID()));

        return Optional.empty();
    }

    private double getAlgaeRemovalSpeed() {
        // double elevatorFast = SmartDashboard.getNumber("RemoveAlgae: elevator fast", -0.7);
        // double elevatorSlow = SmartDashboard.getNumber("RemoveAlgae: elevator slow", -0.3);
        // double slowThreshold = SmartDashboard.getNumber("RemoveAlgae: slow threshold", 1.0);
        // double thing = SmartDashboard.getNumber("RemoveAlgae: thing", 0.3);

        Optional<Boolean> high = isHighAlgae();
        final double elevatorFast = -0.7, elevatorSlow = -0.3, slowThreshold = 0.8;

        if (high.isEmpty())
            return elevatorSlow;

        double elevatorPosition = elevator.getElevatorCANcoderPosition();
        if (high.get()) {
            if (Math.abs(elevatorPosition - (ScoreConstants.kElevatorReef2IntakePosition + 0.5)) <= slowThreshold)
                return elevatorSlow;
            return elevatorFast;
        }

        if (Math.abs(elevatorPosition - (ScoreConstants.kElevatorReef1IntakePosition + 0.5)) <= slowThreshold)
            return elevatorSlow;
        return elevatorFast;
    }

    @Override
    public void simulationPeriodic() {

    }

    public boolean isReefScoringState() {
        return requestedSystemState == L1_SCORE || requestedSystemState == L2_SCORE || requestedSystemState == L3_SCORE || requestedSystemState == L4_SCORE;
    }

    public boolean isReefPrepState(){
        return requestedSystemState == L1_PREP || requestedSystemState == L2_PREP || requestedSystemState == L3_PREP || requestedSystemState == L4_PREP;
    }
}
