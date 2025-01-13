// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  private static Superstructure superstructure;

  private SuperstructureState systemState;
  private SuperstructureState nextSystemState;
  private SuperstructureState requestedSystemState;

  private boolean algaeIndex, coralIndex; // get from intakes :) - not yet set up... :/

  public enum SuperstructureState {
    STOW,
    HP_INTAKE,
    CORAL_GROUND_INTAKE,
    ALGAE_GROUND_INTAKE,
    L1_PREP,
    L2_PREP,
    L3_PREP,
    L4_PREP,
    L1_SCORE,
    L2_SCORE,
    L3_SCORE,
    L4_SCORE,
    CLIMB_PREP,
    CLIMB,
    BARGE_PREP,
    BARGE_SCORE,
    PROCESSOR_PREP,
    PROCESSOR_SCORE,
    REEF1_INTAKE,
    REEF2_INTAKE,
    EJECT_ALGAE,
    EJECT_CORAL
  }

  public Superstructure() {
    systemState = SuperstructureState.STOW;
    nextSystemState = SuperstructureState.STOW;
    requestedSystemState = SuperstructureState.STOW;
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

  @Override
  public void periodic() {
    switch (systemState) {
      case STOW:

        if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L1_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L2_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L3_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L4_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CLIMB_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.PROCESSOR_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF1_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF2_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_ALGAE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_CORAL) {
          nextSystemState = requestedSystemState;
        }

        break;

      case HP_INTAKE:
        break;

      case CORAL_GROUND_INTAKE:
        break;

      case ALGAE_GROUND_INTAKE:
        break;

      case L1_PREP:
        break;

      case L2_PREP:
        break;

      case L3_PREP:
        break;

      case L4_PREP:
        break;

      case L1_SCORE:
        break;

      case L2_SCORE:
        break;

      case L3_SCORE:
        break;

      case L4_SCORE:
        break;

      case CLIMB_PREP:
        break;

      case CLIMB:
        break;

      case BARGE_PREP:
        break;

      case BARGE_SCORE:
        break;

      case PROCESSOR_PREP:
        break;

      case PROCESSOR_SCORE:
        break;

      case REEF1_INTAKE:
        break;

      case REEF2_INTAKE:
        break;

      case EJECT_ALGAE:
        break;

      case EJECT_CORAL:
        break;
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
