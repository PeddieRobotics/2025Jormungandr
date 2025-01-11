// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.utils.Kraken;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase{
  private static Elevator elevator;
  private Kraken elevatorMotor;
  private CANCoder cancoder;

  private final DCMotor m_elevatorGearbox = DCMotor.getKrakenX60Foc(1);

  public enum elevatorState{
    Stow, L1, L2, L3, L4, Intake
  }

  // Standard classes for controlling our elevator
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
          Constants.kElevatorKp,
          Constants.kElevatorKi,
          Constants.kElevatorKd,
          new TrapezoidProfile.Constraints(2.45, 2.45));
  ElevatorFeedforward m_feedforward =
      new ElevatorFeedforward(
          Constants.kElevatorkS,
          Constants.kElevatorkG,
          Constants.kElevatorkV,
          Constants.kElevatorkA);

  // Simulation classes help us simulate what's going on, including gravity.
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          m_elevatorGearbox,
          Constants.kElevatorGearing,
          Constants.kCarriageMass,
          Constants.kElevatorDrumRadius,
          Constants.kMinElevatorHeightMeters,
          Constants.kMaxElevatorHeightMeters,
          true,
          0);
  
  private final CANCoderSimCollection cancoderSim = cancoder.getSimCollection();
  private final TalonFXSimCollection krakenSim = kraken.getSimCollection();
 
  public Elevator() {
    //set values when have real robot
    elevatorMotor = new Kraken(RobotMap.ELEVATOR_MOTOR, RobotMap.CANIVORE_NAME);
    cancoder = new CANcoder(RobotMap.ELEVATOR_CANCODER_ID, RobotMap.CANIVORE_NAME );
    configureCANcoder();

    SmartDashboard.putData("Elevator Sim", m_mech2d);
    SmartDashboard.putNumber("elevator setpoint",0);
  }

  public configureCANcoder(){
    //review when have real robot
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.SensorDirection = SensorDirection.Clockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = ElevatorConstants.elevatorMagnetOffset;

    cancoder.getConfigurator().apply(canCoderConfig);
  }

  

  public static Elevator getInstance(){
    if(elevator == null){
      elevator = new Elevator();
    }
    return elevator;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
     // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

    setSpeed(SmartDashboard.getNumber("elevator setpoint",0));
  }

  public void setSpeed(double goal) {
    m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput =
        m_feedforward.calculate(MetersPerSecond.of(m_controller.getSetpoint().velocity).magnitude());
    m_motor.setVoltage(pidOutput + feedforwardOutput);
  }
  /** Stop the control loop and motor output. */
  public void stop() {
    m_controller.setGoal(0.0);
    m_motor.set(0.0);
  }

  /** Update telemetry, including the mechanism visualization. */
  public void updateTelemetry() {
    // Update elevator visualization with position
    m_elevatorMech2d.setLength(m_encoder.getDistance());
  }
}
