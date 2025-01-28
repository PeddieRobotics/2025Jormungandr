// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.DrbgParameters.NextBytes;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.ElevatorConstants;
import frc.robot.utils.Constants.ScoreConstants;

public class Superstructure extends SubsystemBase {
  private static Superstructure superstructure;

  private final Arm arm;
  private final Claw claw;
  private final Drivetrain drivetrain;
  private final Elevator elevator;
  private final HPIntake hpIntake;

  private SuperstructureState systemState;
  private SuperstructureState requestedSystemState;

  private boolean algaeIndex, coralIndex; // get from intakes :) - not yet set up... :/

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
    REEF1_INTAKE,
    REEF2_INTAKE,
    EJECT_ALGAE,
    EJECT_CORAL
  }

  private Map<SuperstructureState, Set<SuperstructureState>> transitions = new HashMap<>() {{
    put(SuperstructureState.STOW, Set.of(
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.HP_INTAKE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.ALGAE_GROUND_INTAKE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.L1_PREP, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_SCORE,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.L2_PREP, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_SCORE,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.L3_PREP, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_SCORE,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.L4_PRESTAGE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PREP,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.L4_PREP, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.L4_SCORE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.L1_SCORE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE));

    put(SuperstructureState.L2_SCORE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE));

    put(SuperstructureState.L3_SCORE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE));

    put(SuperstructureState.L4_SCORE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE));

    put(SuperstructureState.BARGE_PRESTAGE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PREP,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.BARGE_PREP, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.BARGE_SCORE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.BARGE_SCORE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE));

    put(SuperstructureState.PROCESSOR_PREP, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_SCORE,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.PROCESSOR_SCORE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE));

    put(SuperstructureState.REEF1_INTAKE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.REEF2_INTAKE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.EJECT_ALGAE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.EJECT_ALGAE, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_CORAL));

    put(SuperstructureState.EJECT_CORAL, Set.of(
        SuperstructureState.STOW,
        SuperstructureState.HP_INTAKE,
        SuperstructureState.ALGAE_GROUND_INTAKE,
        SuperstructureState.L1_PREP,
        SuperstructureState.L2_PREP,
        SuperstructureState.L3_PREP,
        SuperstructureState.L4_PRESTAGE,
        SuperstructureState.BARGE_PRESTAGE,
        SuperstructureState.PROCESSOR_PREP,
        SuperstructureState.REEF1_INTAKE,
        SuperstructureState.REEF2_INTAKE,
        SuperstructureState.EJECT_ALGAE));
  }};

  public Superstructure() {
    systemState = SuperstructureState.STOW;
    requestedSystemState = SuperstructureState.STOW;

    arm = Arm.getInstance();
    claw = Claw.getInstance();
    drivetrain = Drivetrain.getInstance();
    elevator = Elevator.getInstance();
    hpIntake = HPIntake.getInstance();

    timer = new Timer();
    // timer.start();
    algaeIndex = false;
    coralIndex = false;
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

  @Override
  public void periodic() {
    SmartDashboard.putString("current superstructure state", systemState.toString());
    SmartDashboard.putString("requested superstructure state", requestedSystemState.toString());
    algaeIndex = SmartDashboard.getBoolean("algaeIndex", false);
    coralIndex = SmartDashboard.getBoolean("coralIndex", false);

    switch (systemState) {
      case STOW:
        // stop intake
        // bring elevator down
        break;

      case HP_INTAKE:
        // set angle
        // set elevator
        // run intake motor
        break;

      case ALGAE_GROUND_INTAKE:
        // run intake
        if (algaeIndex) {
          requestState(SuperstructureState.STOW);
        }
        break;

      case L1_PREP:
        // set prep angle
        break;

      case L2_PREP:
        // set prep angle
        /*
         * two different cases:
         * - dunk case
         * move to angle close to scoring angle
         * - shoot case
         * move to scoring angle
         */
        break;

      case L3_PREP:
        // set prep angle
        /*
         * two different cases:
         * - dunk case
         * move to angle close to scoring angle
         * - shoot case
         * move to scoring angle
         */
        break;

      case L4_PREP:
        // set prep angle
        /*
         * move elevator to scoring height
         * one case:
         * - dunk/score case
         * move to angle close to scoring angle (vertical)
         */
        break;

      case L1_SCORE:

        if (!timer.hasElapsed(ScoreConstants.L1ScoreTimeout)) {
          // KEEP RUNNING
          timer.start();
        } else if (timer.hasElapsed(ScoreConstants.L1ScoreTimeout) && !coralIndex) {
          timer.reset();

          // stop everything
          requestState(SuperstructureState.STOW);
          break;
        }
        // lower arm :)
        break;

      case L2_SCORE:

        if (!timer.hasElapsed(ScoreConstants.L2ScoreTimeout)) {
          // KEEP RUNNING
          timer.start();
          /*
           * two different cases:
           * - dunk case
           * move to angle and drop
           * - shoot case
           * eject piece
           */

        } else if (timer.hasElapsed(ScoreConstants.L2ScoreTimeout) && !coralIndex) {
          timer.reset();

          // stop everything
          requestState(SuperstructureState.STOW);
          break;
        }
        // lower arm :)
        break;

      case L3_SCORE:
        if (!timer.hasElapsed(ScoreConstants.L3ScoreTimeout)) {
          // KEEP RUNNING
          timer.start();
          /*
           * two different cases:
           * - dunk case
           * move to angle and drop
           * - shoot case
           * eject piece
           */

        } else if (timer.hasElapsed(ScoreConstants.L3ScoreTimeout) && !coralIndex) {
          timer.reset();

          // stop everything
          requestState(SuperstructureState.STOW);
          break;
        }
        // lower arm :)
        break;

      case L4_SCORE:

        if (!timer.hasElapsed(ScoreConstants.L4ScoreTimeout)) {
          // KEEP RUNNING
          /*
           * two different cases:
           * - dunk case
           * move to angle and drop
           * - shoot case
           * move to angle and eject piece
           */
          timer.start();

        } else if (timer.hasElapsed(ScoreConstants.L4ScoreTimeout) && !coralIndex) {
          timer.reset();

          // stop everything
          requestState(SuperstructureState.STOW);
          break;
        }
        // lower arm :)
        break;

      case L4_PRESTAGE:
        elevator.setElevatorPositionMotionMagicTorqueCurrentFOC(ElevatorConstants.kElevatorL3Height);
        break;

      case BARGE_PRESTAGE:
        break;

      case BARGE_PREP:
        break;

      case BARGE_SCORE:
        if (!timer.hasElapsed(ScoreConstants.BargeTimeout)) {
          // KEEP RUNNING
          timer.start();

        } else if (timer.hasElapsed(ScoreConstants.BargeTimeout) && !algaeIndex) {
          timer.reset();

          // stop everything
          requestState(SuperstructureState.STOW);
          break;
        }
        break;

      case PROCESSOR_PREP:
        break;

      case PROCESSOR_SCORE:
        requestState(SuperstructureState.STOW);
        break;

      case REEF1_INTAKE:
        if (algaeIndex) {
          requestState(SuperstructureState.STOW);
        }
        break;

      case REEF2_INTAKE:
        if (algaeIndex) {
          requestState(SuperstructureState.STOW);
        }
        break;

      case EJECT_ALGAE:
        break;

      case EJECT_CORAL:
        break;
    }

    Set<SuperstructureState> canTransition = transitions.get(systemState);
    if (canTransition.contains(requestedSystemState))
      systemState = requestedSystemState;
  }

  public void sendToScore() {
    switch (systemState) {
      case L1_PREP:
        requestState(SuperstructureState.L1_SCORE);
        timer.reset();
        break;

      case L2_PREP:
        requestState(SuperstructureState.L2_SCORE);
        timer.reset();
        break;

      case L3_PREP:
        requestState(SuperstructureState.L3_SCORE);
        timer.reset();
        break;

      case L4_PREP:
        requestState(SuperstructureState.L4_SCORE);
        timer.reset();
        break;

      case PROCESSOR_PREP:
        requestState(SuperstructureState.PROCESSOR_SCORE);
        break;

      case BARGE_PREP:
        requestState(SuperstructureState.BARGE_SCORE);
        break;
    }
  }

  @Override
  public void simulationPeriodic() {

  }
}
