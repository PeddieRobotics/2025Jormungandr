// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClawConstants;
import frc.robot.utils.Constants.ScoreConstants;

import static frc.robot.subsystems.Superstructure.SuperstructureState.*;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;

    private final Arm arm;
    private final Claw claw;
    private final Elevator elevator;
    // private final HPIntake hpIntake;

    private SuperstructureState systemState;
    private SuperstructureState requestedSystemState;

    private boolean algaeIndex, bothCoralSensorsTriggered;

    private Timer timer;

    public enum SuperstructureState {
        STOW,
        HP_INTAKE,
        ALGAE_GROUND_INTAKE,
        L1_PREP,
        L2_PREP,
        L3_PREP,
        L4_PRESTAGE,
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
        EJECT_CORAL
    }

    private boolean clawIncremented = false;

    public Superstructure() {
        systemState = STOW;
        requestedSystemState = STOW;

        arm = Arm.getInstance();
        claw = Claw.getInstance();
        elevator = Elevator.getInstance();
        // hpIntake = HPIntake.getInstance();

        timer = new Timer();
        // timer.start();
        algaeIndex = false;
        bothCoralSensorsTriggered = false;
        SmartDashboard.putBoolean("algaeIndex", false);
        SmartDashboard.putBoolean("coralIndex", false);
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

    @Override
    public void periodic() {
        SmartDashboard.putString("current superstructure state", systemState.toString());
        SmartDashboard.putString("requested superstructure state", requestedSystemState.toString());

        bothCoralSensorsTriggered = claw.bothCoralSensorsTriggered();
        algaeIndex = claw.hasAlgae();

        switch (systemState) {
            case STOW -> {
                // stop intake
                // bring elevator down
                // elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorStowPosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmStowPosition);
                arm.setArmPositionVoltage(ScoreConstants.kArmStowPosition);
                elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorStowPosition);
                if (algaeIndex) {
                    claw.holdAlgae();
                } else {
                    claw.stopClaw();
                }

                if (Arrays.asList(
                        STOW, HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
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
                // elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorHPIntakePosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmHPIntakePosition);

                elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorHPIntakePosition);
                arm.setArmPositionVoltage(ScoreConstants.kArmHPIntakePosition);

                // add gate to check elevator height and arm angle ?

                if(claw.getTopSensor() && claw.getBottomSensor()) {
                    claw.stopClaw();
                    requestState(STOW);
                } else if (claw.getTopSensor() && !claw.getBottomSensor()){
                    claw.intakePiece(ClawConstants.kCoralSlowIntake);
                } else {
                    claw.intakePiece(ClawConstants.kCoralIntakeSpeed);
                }


                if (Arrays.asList(
                        STOW,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
                        L4_PREP,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE,
                        EJECT_CORAL)
                        .contains(requestedSystemState)) {
                    clawIncremented = false;
                    systemState = requestedSystemState;
                }
            }

            case ALGAE_GROUND_INTAKE -> {
                // run intake
                // elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorGroundIntakePosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmGroundIntakePosition);

                // if (algaeIndex) {
                //     claw.holdAlgae();
                //     requestState(HP_INTAKE);
                // }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
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
                // elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorL1ScorePosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmL1ScorePosition);

                elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorL1ScorePosition);
                arm.setArmPositionVoltage(ScoreConstants.kArmL1ScorePosition);

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_SCORE,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
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
                // elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorL2ScorePosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmL2ScorePosition);

                elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorL2ScorePosition);
                arm.setArmPositionVoltage(ScoreConstants.kArmL2ScorePosition);

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_SCORE,
                        L3_PREP,
                        L4_PRESTAGE,
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
                // elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorL3ScorePosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmL3ScorePosition);
                elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorL3ScorePosition);
                arm.setArmPositionVoltage(ScoreConstants.kArmL3ScorePosition);


                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_SCORE,
                        L4_PRESTAGE,
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

            case L4_PRESTAGE -> {
                elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorL4PrestagePosition);
                arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmStowPosition);

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
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

            case L4_PREP -> {
                // set prep angle
                /*
                 * move elevator to scoring height
                 * one case:
                 * - dunk/score case
                 * move to angle close to scoring angle (vertical)
                 */
                // elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorL4ScorePosition);
                // arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmL4ScorePosition);
                elevator.setElevatorPositionVoltage(ScoreConstants.kElevatorL4ScorePosition);
                arm.setArmPositionVoltage(ScoreConstants.kArmL4ScorePosition);

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

                if (!timer.hasElapsed(ScoreConstants.kL1ScoreTimeout)) {
                    // KEEP RUNNING
                    timer.start();
                    claw.outtakePiece();
                } else if (timer.hasElapsed(ScoreConstants.kL1ScoreTimeout) || !claw.eitherCoralSensorTriggered()) {
                    timer.reset();

                    // stop everything
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                }
                // lower arm :)

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

                if (!timer.hasElapsed(ScoreConstants.kL2ScoreTimeout)) {
                    // KEEP RUNNING
                    timer.start();
                    claw.outtakePiece();
                    /*
                     * two different cases:
                     * - dunk case
                     * move to angle and drop
                     * - shoot case
                     * eject piece
                     */

                } else if (timer.hasElapsed(ScoreConstants.kL2ScoreTimeout) || !claw.eitherCoralSensorTriggered()) {
                    timer.reset();

                    // stop everything
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                }
                // lower arm :)

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
                if (!timer.hasElapsed(ScoreConstants.kL3ScoreTimeout)) {
                    // KEEP RUNNING
                    timer.start();
                    claw.outtakePiece();
                    /*
                     * two different cases:
                     * - dunk case
                     * move to angle and drop
                     * - shoot case
                     * eject piece
                     */

                } else if (timer.hasElapsed(ScoreConstants.kL3ScoreTimeout) || !claw.eitherCoralSensorTriggered()) {
                    timer.reset();

                    // stop everything
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                }
                // lower arm :)

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

                if (!timer.hasElapsed(ScoreConstants.kL4ScoreTimeout)) {
                    // KEEP RUNNING
                    /*
                     * two different cases:
                     * - dunk case
                     * move to angle and drop
                     * - shoot case
                     * move to angle and eject piece
                     */
                    timer.start();
                    claw.outtakePiece();

                } else if (timer.hasElapsed(ScoreConstants.kL4ScoreTimeout) || !claw.eitherCoralSensorTriggered()) {
                    timer.reset();

                    // stop everything
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                }
                // lower arm :)

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

            case BARGE_PRESTAGE -> {

                elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorBargePrestagePosition);
                arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmStowPosition);

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
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

                elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmBargeScorePosition);
                arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmBargeScorePosition);

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
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
                if (!timer.hasElapsed(ScoreConstants.kBargeTimeout)) {
                    // KEEP RUNNING
                    timer.start();
                    claw.outtakePiece();

                } else if (timer.hasElapsed(ScoreConstants.kBargeTimeout) || !algaeIndex) {
                    timer.reset();

                    // stop everything
                    claw.stopClaw();
                    requestState(HP_INTAKE);
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
                elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorProcessorScorePosition);
                arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmProcessorScorePosition);

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
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
                if (!timer.hasElapsed(ScoreConstants.kProcessorTimeout)) {
                    // KEEP RUNNING
                    timer.start();
                    claw.outtakePiece();

                } else if (timer.hasElapsed(ScoreConstants.kProcessorTimeout) || !algaeIndex) {
                    timer.reset();

                    // stop everything
                    claw.stopClaw();
                    requestState(HP_INTAKE);
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
                elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorReef1IntakePosition);
                arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmReef1IntakePosition);
                claw.intakePiece(ClawConstants.kAlgaeIntakeSpeed);

                if (algaeIndex) {
                    claw.holdAlgae();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
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
                elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kElevatorReef2IntakePosition);
                arm.setArmPositionMotionMagicTorqueCurrentFOC(ScoreConstants.kArmReef2IntakePosition);
                claw.intakePiece(ClawConstants.kAlgaeIntakeSpeed);

                if (algaeIndex) {
                    claw.holdAlgae();
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
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
                claw.outtakePiece();

                if (!algaeIndex) {
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
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
                claw.outtakePiece();

                if (!claw.eitherCoralSensorTriggered()) {
                    claw.stopClaw();
                    requestState(HP_INTAKE);
                }

                if (Arrays.asList(
                        STOW,
                        HP_INTAKE,
                        ALGAE_GROUND_INTAKE,
                        L1_PREP,
                        L2_PREP,
                        L3_PREP,
                        L4_PRESTAGE,
                        BARGE_PRESTAGE,
                        PROCESSOR_PREP,
                        REEF1_ALGAE_INTAKE,
                        REEF2_ALGAE_INTAKE,
                        EJECT_ALGAE)
                        .contains(requestedSystemState)) {
                    systemState = requestedSystemState;
                }
            }
        }

    }

    public void sendToScore() {
        switch (systemState) {
            case L1_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmL1ScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorL1ScorePosition)) {
                    requestState(L1_SCORE);
                    timer.reset();
                }
            }

            case L2_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmL2ScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorL2ScorePosition)) {
                    requestState(L2_SCORE);
                    timer.reset();
                }
            }

            case L3_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmL3ScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorL3ScorePosition)) {
                    requestState(L3_SCORE);
                    timer.reset();
                }
            }

            case L4_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmL4ScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorL4ScorePosition)) {
                    requestState(L4_SCORE);
                    timer.reset();
                }
            }

            case PROCESSOR_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmProcessorScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorProcessorScorePosition)) {
                    requestState(PROCESSOR_SCORE);
                    timer.reset();
                }
            }

            case BARGE_PREP -> {
                if (arm.isAtPosition(ScoreConstants.kArmBargeScorePosition)
                        && elevator.isAtPosition(ScoreConstants.kElevatorBargeScorePosition)) {
                    requestState(BARGE_SCORE);
                    timer.reset();
                }
            }
            default -> {
                break;
            }
        }
    }

    @Override
    public void simulationPeriodic() {

    }
}
