// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private SuperstructureState nextSystemState;
  private SuperstructureState requestedSystemState;

  private boolean algaeIndex, coralIndex; // get from intakes :) - not yet set up... :/

  private Timer timer;

  public enum SuperstructureState {
    STOW,
    HP_INTAKE,
    CORAL_GROUND_INTAKE, //likely not used
    ALGAE_GROUND_INTAKE,
    L1_PREP,
    L2_PREP,
    L3_PREP,
    L4_PREP,
    L3L4_PRESTAGE, 
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

  public Superstructure() {
    systemState = SuperstructureState.STOW;
    nextSystemState = SuperstructureState.STOW;
    requestedSystemState = SuperstructureState.STOW;

    arm = Arm.getInstance();
    claw = Claw.getInstance();
    drivetrain = Drivetrain.getInstance();
    elevator = Elevator.getInstance();
    hpIntake = HPIntake.getInstance();

    timer = new Timer();
    // timer.start();
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
        //stop intake
        //bring elevator down
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
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PRESTAGE) {
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
        //set angle
        //set elevator
        //run intake motor
        if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L1_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L2_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L3_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L4_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PRESTAGE) {
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

      case CORAL_GROUND_INTAKE:
        //run intake
        if (coralIndex) {
          //stop intaking
        }
        break;

      case ALGAE_GROUND_INTAKE:
        //run intake
        if (algaeIndex) {
          //stop intaking
        }
        break;

      case L1_PREP:
        //set prep angle 
        if (requestedSystemState == SuperstructureState.STOW) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L1_SCORE) { //may add another logic gate (?)
          timer.reset();
          nextSystemState = SuperstructureState.L1_SCORE; 
        } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L2_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF1_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF2_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_ALGAE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_CORAL) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L1_SCORE) {
          nextSystemState = requestedSystemState;
        }

        break;

      case L2_PREP:
        //set prep angle 
        /*
         * two different cases:
         * - dunk case
         *     move to angle close to scoring angle
         * - shoot case
         *     move to scoring angle
         */
        if (requestedSystemState == SuperstructureState.STOW) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L2_SCORE) { //may add another logic gate (?)
          timer.reset();
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L1_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF1_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF2_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_ALGAE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_CORAL) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L2_SCORE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        }

        break;

      case L3_PREP:  
        //set  prep angle 
        /*
         * two different cases:
         * - dunk case
         *     move to angle close to scoring angle
         * - shoot case
         *     move to scoring angle
         */
          if (requestedSystemState == SuperstructureState.HP_INTAKE) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.L3_SCORE) { //may add another logic gate (?)
            timer.reset();
          } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.L1_PREP) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.L2_PREP) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.L4_PREP) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.REEF1_INTAKE) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.REEF2_INTAKE) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.EJECT_ALGAE) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.EJECT_CORAL) {
            nextSystemState = requestedSystemState;
          } else if (requestedSystemState == SuperstructureState.STOW) {
            nextSystemState = requestedSystemState;
          }
        break;

      case L4_PREP:
        //set prep angle 
        /*
         * move elevator to scoring height
         * one case:
         * - dunk/score case
         *     move to angle close to scoring angle (vertical)
         */

        if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L3_SCORE) { //may add another logic gate (?)
            timer.reset();
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
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF1_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF2_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_ALGAE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_CORAL) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.STOW) {
          nextSystemState = requestedSystemState;
        }
        break;

      case L1_SCORE: 

        if (!timer.hasElapsed(ScoreConstants.L1ScoreTimeout)) {
          //KEEP RUNNING
          timer.start();

        } else if (timer.hasElapsed(ScoreConstants.L1ScoreTimeout) && !coralIndex){
          timer.reset(); 

          //stop everything
          requestState(SuperstructureState.STOW);
          break;
        }
        if (requestedSystemState == SuperstructureState.STOW) { 
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
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

      case L2_SCORE:

        if (!timer.hasElapsed(ScoreConstants.L2ScoreTimeout)) {
          //KEEP RUNNING
          timer.start();
        /*
         * two different cases:
         * - dunk case
         *     move to angle and drop
         * - shoot case
         *     eject piece
         */

        } else if (timer.hasElapsed(ScoreConstants.L2ScoreTimeout) && !coralIndex){
          timer.reset(); 

          //stop everything
          requestState(SuperstructureState.STOW);
          break;
        }

        if (requestedSystemState == SuperstructureState.STOW) {
          nextSystemState = requestedSystemState;
        
        } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
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

      case L3_SCORE:
        if (!timer.hasElapsed(ScoreConstants.L3ScoreTimeout)) {
          //KEEP RUNNING
          timer.start();
        /*
         * two different cases:
         * - dunk case
         *     move to angle and drop
         * - shoot case
         *     eject piece
         */

        } else if (timer.hasElapsed(ScoreConstants.L3ScoreTimeout) && !coralIndex){
          timer.reset(); 

          //stop everything
          requestState(SuperstructureState.STOW);
          break;
        }

        if (requestedSystemState == SuperstructureState.STOW) {
          nextSystemState = requestedSystemState;
        
        } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
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

      case L4_SCORE:

        /*
         * two different cases:
         * - dunk case
         *     move to angle and drop
         * - shoot case
         *     move to angle and eject piece
         */

        if (!timer.hasElapsed(ScoreConstants.L4ScoreTimeout)) {
          //KEEP RUNNING
          timer.start();

        } else if (timer.hasElapsed(ScoreConstants.L4ScoreTimeout) && !coralIndex){
          timer.reset(); 

          //stop everything
          requestState(SuperstructureState.STOW);
          break;
        }

        if (requestedSystemState == SuperstructureState.STOW) {
          nextSystemState = requestedSystemState;
        
        } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PRESTAGE) {
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

      case L3L4_PRESTAGE:
        elevator.setElevatorPositionMotionMagic(ElevatorConstants.kElevatorL3Height);

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
        } else if (requestedSystemState == SuperstructureState.L3_PREP) { //check for any other logic
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L4_PREP) { //check for any other logic
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PRESTAGE) {
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
        } else if (requestedSystemState == SuperstructureState.L3_SCORE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L4_SCORE) {
          nextSystemState = requestedSystemState;
        }
        break;


      case BARGE_PRESTAGE:
        if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L1_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L2_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
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

      case BARGE_PREP:
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
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_SCORE) {
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

      case BARGE_SCORE:

        if (!timer.hasElapsed(ScoreConstants.BargeTimeout)) {
          //KEEP RUNNING
          timer.start();

        } else if (timer.hasElapsed(ScoreConstants.BargeTimeout) && !algaeIndex){
          timer.reset(); 

          //stop everything
          requestState(SuperstructureState.STOW);
          break;
        } 

        if (requestedSystemState == SuperstructureState.STOW) {
          nextSystemState = requestedSystemState;
        
        } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
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

      case PROCESSOR_PREP:
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
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.PROCESSOR_SCORE) {
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

      case PROCESSOR_SCORE:
        requestState(SuperstructureState.STOW);
        if (requestedSystemState == SuperstructureState.STOW) {
          nextSystemState = requestedSystemState;
      
        } else if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
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

      case REEF1_INTAKE:
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
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.PROCESSOR_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF2_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_ALGAE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_CORAL) {
          nextSystemState = requestedSystemState;
        }

        break;

      case REEF2_INTAKE:
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
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.PROCESSOR_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF1_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_ALGAE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_CORAL) {
          nextSystemState = requestedSystemState;
        }

        break;

      case EJECT_ALGAE:
        if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.CORAL_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.ALGAE_GROUND_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.BARGE_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.PROCESSOR_PREP) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF1_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF2_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_CORAL) {
          nextSystemState = requestedSystemState;
        }

        break;

      case EJECT_CORAL:
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
        } else if (requestedSystemState == SuperstructureState.L3L4_PRESTAGE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF1_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.REEF2_INTAKE) {
          nextSystemState = requestedSystemState;
        } else if (requestedSystemState == SuperstructureState.EJECT_ALGAE) {
          nextSystemState = requestedSystemState;
        }
        break;
    }
    systemState = nextSystemState;
  }

  public void sendToScore(){
    switch (systemState) {
      case L1_PREP:
        requestState(SuperstructureState.L1_SCORE);
        break;
  
      case L2_PREP:
        requestState(SuperstructureState.L2_SCORE);
        break;

      case L3_PREP:
        requestState(SuperstructureState.L3_SCORE);
        break;
    
      case L4_PREP:
        requestState(SuperstructureState.L4_SCORE);   
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
