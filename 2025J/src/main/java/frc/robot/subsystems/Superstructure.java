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
        break;

      case ALGAE_GROUND_INTAKE:
        break;

      case L1_PREP:
        if (requestedSystemState == SuperstructureState.HP_INTAKE) {
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
        if (requestedSystemState == SuperstructureState.HP_INTAKE) {
          nextSystemState = requestedSystemState;
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
        }
        break;

      case L3_PREP:  
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
          }
        break;

      case L4_PREP:
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
        }
        break;

      case L1_SCORE: 
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

      case L2_SCORE:
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

      case L3_SCORE:
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

      case L4_SCORE:
        requestState(SuperstructureState.STOW);
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
  }

  @Override
  public void simulationPeriodic() {
    
  }
}
