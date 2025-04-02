package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Kraken {
    private final TalonFX talon;
    // configurator for TalonFX
    private TalonFXConfiguration config;
    private int deviceID;
    private String canbusName;

    // Open Loop Control
    private double feedForward = 0.0;
    private double velocityConversionFactor = 1.0;

    public Kraken(int deviceID, String canbusName) {
        this.talon = new TalonFX(deviceID, canbusName);
        factoryReset();
        this.deviceID = deviceID;
        this.canbusName = canbusName;
        config = new TalonFXConfiguration();
        talon.getConfigurator().setPosition(0);
    }

    /**
     * completely reset motor configuration to default (do on deploy to make sure no random settings are unchecked)
     */
    public void factoryReset() {
        talon.getConfigurator().apply(new TalonFXConfiguration());
    }

    /** 
     * set kraken encoder to a given position
     * 
     * @param position - position in motor encoder units (may vary)
     */ 
    public void setEncoder(double position) {
        talon.getConfigurator().setPosition(position);
    }

    /** 
     * set motor to brake mode
     */ 
    public void setBrake() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talon.getConfigurator().apply(config);
    }

    /** 
     * set motor to coast mode
     */ 
    public void setCoast() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talon.getConfigurator().apply(config);
    }

    /** 
     * @return Position of the device in mechanism rotations. This can be the position of a remote sensor and is affected by the RotorToSensorRatio and SensorToMechanismRatio configs, as well as calls to setPosition.
     */ 
    public double getPosition() {
        return talon.getPosition().getValueAsDouble();
    }

    /** 
     * @return motor percent output setpoint [-1.0, 1.0]
     */
    public double getPercentOutput() {
        return talon.get();
    }

    /** 
     * @return motor velocity in meter/s (used for swerve module to determine floor speed post velocity conversion factor)
     */
    public double getMPS() {
        return talon.getRotorVelocity().getValueAsDouble() * velocityConversionFactor;
    }

    /** 
     * @return motor rotor velocity (motor rotations/second)(without conversion factor/feedback configs)
     */
    public double getRPS() {
        return talon.getRotorVelocity().getValueAsDouble();
    }

    /** 
     * @return motor rotor velocity RPMs (motor rotations/minute)(without conversion factor/feedback configs)
     */ 
    public double getRPM() {
        return talon.getRotorVelocity().getValueAsDouble() * 60.0;
    }

    /** 
     * Set motor supply current limit (maximum current draw from BATTERY, helps to reduce brownouts)
     * @param currentLimit - motor supply current limit (amps)
     */ 
    public void setSupplyCurrentLimit(double currentLimit) {
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;

        talon.getConfigurator().apply(config);
    }

    /** 
     * Set motor stator current limit (maximum current draw from MOTOR, helps to find stall current to prevent slipping)
     * @param currentLimit - motor stator current limit (amps)
     */ 
    public void setStatorCurrentLimit(double currentLimit){
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = currentLimit;

        talon.getConfigurator().apply(config);
    }

    /** 
     * Set motor forward torque current limit (maximum current draw from MOTOR in TorqueCurrentFOC control modes)
     * @param currentLimit - motor forward TorqueCurrent limit (amps)
     */ 
    public void setForwardTorqueCurrentLimit(double currentLimit) {
        config.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
        talon.getConfigurator().apply(config);
    }

    /** 
     * Set motor reverse torque current limit (maximum current draw from MOTOR in TorqueCurrentFOC control modes)
     * @param currentLimit - motor reverse TorqueCurrent limit (amps)
     */
    public void setReverseTorqueCurrentLimit(double currentLimit) {
        config.TorqueCurrent.PeakReverseTorqueCurrent = currentLimit;
        talon.getConfigurator().apply(config);
    }
 
    /** 
     * Set whether the motor is inverted
     * @param inverted - true (inverted, clockwise positive), false (not inverted, counterclockwise positive)
     */
    public void setInverted(boolean inverted) {
        if (inverted) {
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        // talon.setInverted(inverted);
        talon.getConfigurator().apply(config);
    }

    /** 
     * Set a ramp rate for closed loop motor control (time to go from 0 output to max output) 
     * (applies to DutyCycle, TorqueCurrent, and Voltage control modes)
     * @param rampRate - ramp rate (seconds)
     */
    public void setClosedLoopRampRate(double rampRate) {
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRate;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRate;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

        talon.getConfigurator().apply(config);
    }

    /** 
     * Set motion magic parameters for motion magic closed loop control
     * @param cruiseVelocity - target/max motion magic motor velocity (CANcoder/mechanism rot/s (depends on feedback device and conversion factors))
     * @param maxAcceleration - target/max motion magic motor acceleration (CANcoder/mechanism rot/s^2 (depends on feedback device and conversion factors))
     * @param maxJerk - target/max motion magic motor jerk (derivative of velocity)(CANcoder/mechanism rot/s^3 (depends on feedback device and conversion factors))
     */
    public void setMotionMagicParameters(double cruiseVelocity, double maxAcceleration, double maxJerk) {
        config.MotionMagic.MotionMagicJerk = maxJerk;
        config.MotionMagic.MotionMagicAcceleration = maxAcceleration;
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;

        talon.getConfigurator().apply(config);
    }

    /** 
     * @return PID slot 0 kS
     */
    public double getKS() {
        return config.Slot0.kS;
    }

    /** 
     * @return PID slot 0 kV
     */
    public double getKV() {
        return config.Slot0.kV;
    }

    /** 
     * @return PID slot 0 kA
     */
    public double getKA() {
        return config.Slot0.kA;
    }

    /** 
     * @return PID slot 0 kP
     */
    public double getKP() {
        return config.Slot0.kP;
    }

    /** 
     * @return PID slot 0 kI
     */
    public double getKI() {
        return config.Slot0.kI;
    }

    /** 
     * @return PID slot 0 kD
     */
    public double getKD() {
        return config.Slot0.kD;
    }

    /** 
     * @return target/maximum motion magic acceleration (CANcoder/mechanism rot/s^2 (depends on feedback device and conversion factors))
     */
    public double getMotionMagicMaxAccel() {
        return config.MotionMagic.MotionMagicAcceleration;
    }

    /** 
     * @return target/maximum motion magic jerk (CANcoder/mechanism rot/s^3 (depends on feedback device and conversion factors))
     */
    public double getMotionMagicMaxJerk() {
        return config.MotionMagic.MotionMagicJerk;
    }

    /** 
     * @return target/maximum motion magic velocity (CANcoder/mechanism rot/s (depends on feedback device and conversion factors))
     */
    public double getKMaxCruiseVelocity() {
        return config.MotionMagic.MotionMagicCruiseVelocity;
    }

    /** 
     * Set forward and reverse soft limits on motor
     * @param enableSoftLimit - enable (true) or disable (false) soft limits
     * @param forwardLimitValue - forward motor rotation limit (post conversion factors))
     * @param reverseLimitValue - reverse motor rotation limit (post conversion factors))
     */
    public void setSoftLimits(boolean enableSoftLimit, double forwardLimitValue, double reverseLimitValue) {

        if (enableSoftLimit) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimitValue;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimitValue;
        } else {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }

        talon.getConfigurator().apply(config);
    }

    /** 
     * Set motor encoder to wrap position error within [-0.5,+0.5) mechanism rotations. 
     * Typically used for continuous position closed-loops like swerve azimuth. (depends on feedback device)
     */
    public void setContinuousOutput() {
        config.ClosedLoopGeneral.ContinuousWrap = true;
        talon.getConfigurator().apply(config);
    }

    /**
     * set mechanism position of motor encoder to 0
     */
    public void resetEncoder() {
        talon.getConfigurator().setPosition(0);
    }

    /**
     * Set motor in open loop control to percent output. This control mode will output a given proportion of supply voltage.
     * @param percentOutput - Proportion of supply voltage to apply to motor [-1.0,1.0]
     */
    public void setPercentOutput(double percentOutput) {
        final DutyCycleOut request = new DutyCycleOut(0);
        // Ensure the percentOutput is within the acceptable range [-1.0, 1.0]
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));

        // Set the control request to the motor controller
        talon.setControl(request.withOutput(percentOutput));
    }

    /**
     * Set a motor to follow a different (master) motor
     * @param masterCANId - CAN ID of master motor to follow
     * @param inverted - Set to false for motor invert to match the master's configured Invert - which is typical when master and follower are mechanically linked and spin in the same direction. Set to true for motor invert to oppose the master's configured Invert - this is typical where the the master and follower mechanically spin in opposite directions.
     */
    public void setFollower(int masterCANId, boolean inverted) {
        talon.setControl(new Follower(masterCANId, inverted));
    }

    /**
     * set PID values for closed loop setpoint control
     * @param kP - proportional gain (units vary)
     * @param kI - integral gain (units vary)
     * @param kD - derivative gain (units vary)
     * @param kF - constant feedforward to apply (volts)
     */
    public void setPIDValues(double kP, double kI, double kD, double kF) {

        feedForward = kF;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        talon.getConfigurator().apply(config);
    }

    /**
     * set PID values for closed loop setpoint control
     * @param kS - static feedforward gain to overcome static friction (units vary)
     * @param kV - velocity feedforward gain (units vary)
     * @param kA - acceleration feedforward gain (units vary)
     * @param kP - proportional gain (units vary)
     * @param kI - integral gain (units vary)
     * @param kD - derivative gain (units vary)
     * @param kF - constant feedforward to apply (volts)
     */
    public void setPIDValues(double kS, double kV, double kA, double kP, double kI, double kD, double kF) {
        feedForward = kF;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        talon.getConfigurator().apply(config);
    }

    /**
     * set PID values for closed loop setpoint control (with kG gravity feedforward)
     * @param kS - static feedforward gain to overcome static friction (units vary)
     * @param kV - velocity feedforward gain (units vary)
     * @param kA - acceleration feedforward gain (units vary)
     * @param kP - proportional gain (units vary)
     * @param kI - integral gain (units vary)
     * @param kD - derivative gain (units vary)
     * @param kF - constant feedforward to apply (volts)
     * @param kG - gravity feedforward/feedback gain (units vary). This is added to the closed loop output. The sign is determined by GravityType. The unit for this constant is dependent on the control mode, typically fractional duty cycle, voltage, or torque current.
     * @param gravityType - This determines the type of the gravity feedforward/feedback. 
     *                      <p> Choose Elevator_Static for systems where the gravity feedforward is constant, such as an elevator. The gravity feedforward output will always have the same sign.
     *                      <p> Choose Arm_Cosine for systems where the gravity feedback is dependent on the angular position of the mechanism, such as an arm. The gravity feedback output will vary depending on the mechanism angular position. Note that the sensor offset and ratios must be configured so that the sensor reports a position of 0 when the mechanism is horizonal (parallel to the ground), and the reported sensor position is 1:1 with the mechanism.
     */
    public void setPIDValues(double kS, double kV, double kA, double kP, double kI, double kD, double kF, double kG, GravityTypeValue gravityType) {
        config.Slot0.GravityType = gravityType;

        feedForward = kF;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kG = kG;
        talon.getConfigurator().apply(config);
    }

    /**
     * set PID values for closed loop setpoint control (with kS feedforward sign)
     * @param kS - static feedforward gain to overcome static friction (units vary)
     * @param kV - velocity feedforward gain (units vary)
     * @param kA - acceleration feedforward gain (units vary)
     * @param kP - proportional gain (units vary)
     * @param kI - integral gain (units vary)
     * @param kD - derivative gain (units vary)
     * @param kF - constant feedforward to apply (volts)
     * @param feedforwardSign - Static Feedforward Sign during position closed loop.
     *                          <p>
     *                          This determines the sign of the applied kS during position
     *                          closed-loop modes. The default behavior uses the velocity reference
     *                          sign. This works well with velocity closed loop, Motion Magic®
     *                          controls, and position closed loop when velocity reference is
     *                          specified (motion profiling).
     *                          <p>
     *                          However, when using position closed loop with zero velocity
     *                          reference (no motion profiling), the application may want to apply
     *                          static feedforward based on the sign of closed loop error instead.
     *                          When doing so, we recommend using the minimal amount of kS,
     *                          otherwise the motor output may dither when closed loop error is
     *                          near zero.
     */
    public void setPIDValues(double kS, double kV, double kA, double kP, double kI, double kD, double kF, StaticFeedforwardSignValue feedforwardSign) {
        config.Slot0.StaticFeedforwardSign = feedforwardSign;

        feedForward = kF;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        talon.getConfigurator().apply(config);
    }

    public void setPIDValues(double kS, double kV, double kA, double kP, double kI, double kD, double kF, double kG, GravityTypeValue gravityType, StaticFeedforwardSignValue feedforwardSign) {
        config.Slot0.StaticFeedforwardSign = feedforwardSign;
        config.Slot0.GravityType = gravityType;

        feedForward = kF;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kG = kG;
        talon.getConfigurator().apply(config);
    }

    public void setPIDValuesSlot1(double kS, double kV, double kA, double kP, double kI, double kD, double kF, double kG, GravityTypeValue gravityType, StaticFeedforwardSignValue feedforwardSign) {
        config.Slot1.StaticFeedforwardSign = feedforwardSign;
        config.Slot1.GravityType = gravityType;

        feedForward = kF;
        config.Slot1.kS = kS;
        config.Slot1.kV = kV;
        config.Slot1.kA = kA;
        config.Slot1.kP = kP;
        config.Slot1.kI = kI;
        config.Slot1.kD = kD;
        config.Slot1.kG = kG;
        talon.getConfigurator().apply(config);
    }
    
    //DO NOT AUTOFORMAT
    /**
     *                Motor and CANcoder to Mechanism overview
     * 
     *            +------------------+         +-------------------+
     *            |                  |         |                   |
     *            |      Motor       |         |     Mechanism     |
     *            |   (Rotor Shaft)  |         |   (CANcoder, if   |
     *            |                  |         |     applicable)   |
     *            |                  |         |                   |
     *            +--------+---------+         +---------+---------+
     *                     |                             |
     *                     v                             v
     *             Rotor to Mechanism          CANcoder to Mechanism
     *                   (x:1)                         (y:1)
     * 
     *   -----------------------------------------------------------------
     *   │   When Using    | RotorToSensorRatio | SensorToMechanismRatio |
     *   │-----------------|--------------------|------------------------|
     *   │ Fused CANcoder  |         x          |           y            |
     *   |                 | Rotor -> CANcoder  | CANcoder -> mechanism  |
     *   |-----------------|--------------------|------------------------|
     *   | Internal Sensor |        n/a         |           x            |
     *   |                 |  rotor is sensor   |   rotor -> mechanism   |
     *   |-----------------|--------------------|------------------------|
     *   | Remote CANcoder |        n/a         |           y            |
     *   |                 |  rotor is unused   | CANcoder -> mechanism  |
     *   -----------------------------------------------------------------
     */

    /**
     * Set the SensorToMechanismRatio - used for converting sensor (encoder) rotations to mechanism rotations
     * @param conversionFactor - sensor (encoder) rotation to mechanism rotation ratio
     */
    public void setSensorToMechanismRatio(double conversionFactor) {
        config.Feedback.SensorToMechanismRatio = conversionFactor;
        talon.getConfigurator().apply(config);
    }

    /**
     * set velocity conversion factor based on gear ratio 
     * @param conversionFactor - velocity to gear ratio factor (UNITS VARY, ie: may be from rotor to meters/s for swerve control)
     */
    public void setVelocityConversionFactor(double conversionFactor) {
        velocityConversionFactor = conversionFactor;
    }

    /**
     * Set the ratio of motor rotor rotations to sensor rotations (used for FusedCANcoder mode - motor rotor to CANcoder rotations)
     * @param conversionFactor - motor rotor rotation to sensor rotation ratio
     */
    public void setRotorToSensorRatio(double conversionRatio) {
        config.Feedback.RotorToSensorRatio = conversionRatio;
        talon.getConfigurator().apply(config);
    }

    /**
     * Set the motor to its neutral mode, with neutral output and no current draw 
     */
    public void setNeutralControl(){
        final NeutralOut request = new NeutralOut();
        talon.setControl(request);
    }

    /**
     * Request PID to target position with PositionVoltage control mode
     * @param position - motor/mechanism target position setpoint (post conversion factors)
     */
    public void setPositionVoltage(double position) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        talon.setControl(request.withPosition(position).withEnableFOC(true));
    }

    /**
     * Request PID to target velocity with VelocityVoltage control mode
     * @param velocity - motor target velocity setpoint (rot/s)(post velocity conversion factor)
     */
    public void setVelocityVoltage(double velocity) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        talon.setControl(request.withVelocity(velocity / velocityConversionFactor).withEnableFOC(true));
    }

    /**
     * Request PID to target position with PositionVoltage control mode and constant voltage feedforward
     * @param position - motor/mechanism target position setpoint (post conversion factors)
     */
    public void setPositionVoltageWithFeedForward(double position) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        talon.setControl(request.withPosition(position).withFeedForward(feedForward).withEnableFOC(true));
    }

    /**
     * Request PID to target velocity with VelocityVoltage control mode and constant voltage feedforward
     * @param velocity - motor target velocity setpoint (rot/s)(post velocity conversion factor)
     */
    public void setVelocityVoltageWithFeedForward(double velocity) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        talon.setControl(request.withVelocity(velocity / velocityConversionFactor).withFeedForward(feedForward)
                .withEnableFOC(true));
    }

    /**
     * Request PID to target motor velocity with VelocityTorqueCurrentFOC control mode
     * @param velocity - motor target velocity setpoint (rot/s)(post velocity conversion factor)
     */
    public void setVelocityTorqueCurrentFOC(double velocity) {
        final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);
        talon.setControl(request.withVelocity(velocity / velocityConversionFactor));
    }
    /**
     * Request PID to target position with PositionTorqueCurrentFOC control mode
     * @param position - motor/mechanism target position setpoint (post conversion factors)
     */
    public void setPositionTorqueCurrentFOC(double setpoint) {
        final PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(0).withSlot(0);
        talon.setControl(request.withPosition(setpoint).withFeedForward(feedForward));
    }

    /**
     * Request MotionMagic motion profile to target position with MotionMagicVoltage control mode
     * @param position - motor/mechanism target position setpoint (post conversion factors)
     */
    public void setPositionMotionMagicVoltage(double position, int slot) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(slot);
        talon.setControl(request.withPosition(position).withFeedForward(feedForward).withEnableFOC(true));
    }

    /**
     * Request MotionMagic motion profile to target position with MotionMagicTorqueCurrentFOC control mode
     * @param position - motor/mechanism target position setpoint (post conversion factors)
     */
    public void setPositionMotionMagicTorqueCurrentFOC(double position){
        final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
        talon.setControl(request.withPosition(position).withFeedForward(feedForward));
    }

    /**
     * set the feedback device of motor - often for using external encoder (ie: CANcoders)
     * @param deviceID - device ID of remote device to use. This is not used if the Sensor Source is the internal rotor sensor.
     * @param feedbackType - Choose what sensor source is reported via API and used by
     * closed-loop and limit features.  The default is RotorSensor, which
     * uses the internal rotor sensor in the Talon.
     * <p>
     * Choose Remote* to use another sensor on the same CAN bus (this also
     * requires setting FeedbackRemoteSensorID).  Talon will update its
     * position and velocity whenever the remote sensor publishes its
     * information on CAN bus, and the Talon internal rotor will not be
     * used.
     * <p>
     * Choose Fused* and Talon will fuse another
     * sensor's information with the internal rotor, which provides the
     * best possible position and velocity for accuracy and bandwidth
     * (this also requires setting FeedbackRemoteSensorID).  This was
     * developed for applications such as swerve-azimuth.
     * <p>
     * Choose Sync* and Talon will synchronize its
     * internal rotor position against another sensor, then continue to
     * use the rotor sensor for closed loop control (this also requires
     * setting FeedbackRemoteSensorID).  The Talon will report if its
     * internal position differs significantly from the reported remote
     * sensor position.  This was developed for mechanisms where there is
     * a risk of the sensor failing in such a way that it reports a
     * position that does not match the mechanism, such as the sensor
     * mounting assembly breaking off.
     * <p>
     * Choose RemotePigeon2_Yaw, RemotePigeon2_Pitch, and
     * RemotePigeon2_Roll to use another Pigeon2 on the same CAN bus (this
     * also requires setting FeedbackRemoteSensorID).  Talon will update
     * its position to match the selected value whenever Pigeon2 publishes
     * its information on CAN bus. Note that the Talon position will be in
     * rotations and not degrees.
     * <p>
     * Note: When the feedback source is changed to Fused* or Sync*, the
     * Talon needs a period of time to fuse before sensor-based
     * (soft-limit, closed loop, etc.) features are used. This period of
     * time is determined by the update frequency of the remote sensor's
     * Position signal.
     * 
     */
    public void setFeedbackDevice(int deviceID, FeedbackSensorSourceValue feedbackType) {
        config.Feedback.FeedbackRemoteSensorID = deviceID;
        config.Feedback.FeedbackSensorSource = feedbackType;
        talon.getConfigurator().apply(config);
    }

    /**
     * @return current supplied by battery to motor (amps)
     */
    public double getSupplyCurrent() {
        return talon.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * @return current used by stator windings of motor (motor/torque current draw). 
     * Stator current where Positive current indicates motoring regardless of direction. Negative current indicates regenerative braking regardless of direction.
     */
    public double getStatorCurrent() {
        return talon.getStatorCurrent().getValueAsDouble();
    }

    /**
     * @return current corresponding to the torque output of the motor (sort of similar to stator)(for torqueCurrenFOC control modes)(amps)
     */
    public double getTorqueCurrent(){
        return talon.getTorqueCurrent().getValueAsDouble();
    }

    /**
     * @return temperature of motor (celcius)
     */
    public double getMotorTemperature() {
        return talon.getDeviceTemp().getValueAsDouble();
    }

    /**
     * put info about motor onto smart dashboard, including CANbus name and device CANId (diagnostics)
     */
    public void updateSmartDashboard() {
        SmartDashboard.putNumber(canbusName + " " + deviceID + " motor", 1);
    }
}