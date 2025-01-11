package frc.robot.utils;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;

public class Kraken {
    private final TalonFX talon;
    // configurator for TalonFX
    private TalonFXConfiguration config;
    private int deviceID;
    private String canbusName;
    private Orchestra orchestra;

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
        
        orchestra = new Orchestra();
        orchestra.addInstrument(talon);
        orchestra.loadMusic("output.chrp");
    }

    // completely reset motor configuration to default
    public void factoryReset() {
        talon.getConfigurator().apply(new TalonFXConfiguration());
    }

    // set kraken encoder to a given position
    public void setEncoder(double position) {
        talon.getConfigurator().setPosition(position);
    }

    // set brake mode
    public void setBrake() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talon.getConfigurator().apply(config);
    }

    // set coast mode
    public void setCoast() {
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talon.getConfigurator().apply(config);
    }

    // get internal motor encoder position reading
    public double getPosition() {
        // should apply position conversion factor after
        return talon.getPosition().getValueAsDouble();
    }

    // get motor velocity setpoint (-1 to 1)
    public double getVelocity() {
        return talon.get();
    }

    // get the motor rotor velocity from TalonFX in meters/second (must apply
    // conversion factor)
    public double getMPS() {
        return talon.getRotorVelocity().getValueAsDouble() * velocityConversionFactor;
    }

    // get the motor rotor velocity in RPMs (must apply conversion factor?)
    public double getRPM() {
        return talon.getRotorVelocity().getValueAsDouble() * 60.0;
        // return talon.getRotorVelocity().getValueAsDouble() *
        // Constants.FlywheelConstants.kFlywheelGearReduction * 60.0;
    }

    // set the current limit of motor
    public void setSupplyCurrentLimit(double currentLimit) {
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        // config.CurrentLimits.StatorCurrentLimitEnable = true;
        // config.CurrentLimits.StatorCurrentLimit = currentLimit;

        talon.getConfigurator().apply(config);
    }

    public void setForwardTorqueCurrentLimit(double currentLimit) {
        config.TorqueCurrent.PeakForwardTorqueCurrent = currentLimit;
        talon.getConfigurator().apply(config);
    }

    public void setReverseTorqueCurrentLimit(double currentLimit) {
        config.TorqueCurrent.PeakReverseTorqueCurrent = currentLimit;
        talon.getConfigurator().apply(config);
    }

    // invert motor
    public void setInverted(boolean inverted) {
        // if(inverted) config.MotorOutput.Inverted =
        // InvertedValue.CounterClockwise_Positive;
        // else config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        if (inverted) {
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        // talon.setInverted(inverted);
        talon.getConfigurator().apply(config);
    }

    // set a ramp rate for closed loop PID control
    public void setClosedLoopRampRate(double rampRate) {
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = rampRate;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = rampRate;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampRate;

        talon.getConfigurator().apply(config);
    }

    // set parameters for motion magic - jerk, vel, accel
    public void setMotionMagicParameters(double cruiseVelocity, double maxAcceleration, double maxJerk) {
        config.MotionMagic.MotionMagicJerk = maxJerk;
        config.MotionMagic.MotionMagicAcceleration = maxAcceleration;
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;

        talon.getConfigurator().apply(config);
    }

    public double getKS() {
        return config.Slot0.kS;
    }

    public double getKV() {
        return config.Slot0.kV;
    }

    public double getKA() {
        return config.Slot0.kA;
    }

    public double getKP() {
        return config.Slot0.kP;
    }

    public double getKI() {
        return config.Slot0.kI;
    }

    public double getKD() {
        return config.Slot0.kD;
    }

    public double getKMaxCruiseAccel() {
        return config.MotionMagic.MotionMagicAcceleration;
    }

    public double getKMaxCruiseJerk() {
        return config.MotionMagic.MotionMagicJerk;
    }

    public double getKMaxCruiseVelocity() {
        return config.MotionMagic.MotionMagicCruiseVelocity;
    }

    // set forward and backward soft limits
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

    // allows for continuous wrap of encoder - finds the shortest path to target
    // position
    public void setContinuousOutput() {
        config.ClosedLoopGeneral.ContinuousWrap = true;
        talon.getConfigurator().apply(config);
    }

    // reset encoder value to 0
    public void resetEncoder() {
        talon.getConfigurator().setPosition(0);
    }

    // set the motor in open loop control using percent output
    public void setMotor(double percentOutput) {
        final DutyCycleOut request = new DutyCycleOut(0);
        // Ensure the percentOutput is within the acceptable range [-1.0, 1.0]
        percentOutput = Math.max(-1.0, Math.min(1.0, percentOutput));

        // Set the control request to the motor controller
        talon.setControl(request.withOutput(percentOutput));

    }

    // set motor to follow a different motor
    // set if motor needs to be inverted when following master motor
    public void setFollower(int masterCANId, boolean inverted) {
        talon.setControl(new Follower(masterCANId, inverted));
    }

    // set PID values for position setpoint control
    public void setPIDValues(double kP, double kI, double kD, double kF) {

        feedForward = kF;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        talon.getConfigurator().apply(config);
    }

    // set PID values for velocity setpoint control
    public void setVelocityPIDValues(double kS, double kV, double kA, double kP, double kI, double kD, double kF) {
        feedForward = kF;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        talon.getConfigurator().apply(config);
    }

    public void setVelocityPIDValues(double kS, double kV, double kA, double kP, double kI, double kD, double kF, double kG, GravityTypeValue gravityType) {
        // TODO: update kraken methods to include kG, cosine mode
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

    // set the SensorToMechanismRatio - used for converting sensor (encoder)
    // rotations to mechanism rotations
    public void setSensorToMechanismRatio(double conversionFactor) {
        config.Feedback.SensorToMechanismRatio = conversionFactor;
        talon.getConfigurator().apply(config);
    }

    // set velocity conversion factor based on gear ratio
    public void setVelocityConversionFactor(double conversionFactor) {
        velocityConversionFactor = conversionFactor;
    }

    // Used for FusedCANcoder, set the ratio of motor rotor rotations to rotations
    // of sensor (CANcoder)
    public void setRotorToSensorRatio(double conversionRatio) {
        config.Feedback.RotorToSensorRatio = conversionRatio;
        talon.getConfigurator().apply(config);
    }

    
    // Sets the motor to its neutral mode, doesn't pull any current
    public void setNeutralControl(){
        final NeutralOut request = new NeutralOut();
        talon.setControl(request);
    }

    // Set the motor target position using PID control
    public void setPosition(double position) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        talon.setControl(request.withPosition(position).withEnableFOC(true));
    }

    // set motor target velocity using velocity PID control
    public void setVelocity(double velocity) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        talon.setControl(request.withVelocity(velocity / velocityConversionFactor).withEnableFOC(true));
    }

    // set the motor target position using PID control and feedforward (often to
    // counter gravity)
    public void setPositionWithFeedForward(double position) {
        final PositionVoltage request = new PositionVoltage(0).withSlot(0);
        talon.setControl(request.withPosition(position).withFeedForward(feedForward).withEnableFOC(true));
    }

    // set the motor target velocity using velocity PID control and feedforward
    public void setVelocityWithFeedForward(double velocity) {
        final VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        talon.setControl(request.withVelocity(velocity / velocityConversionFactor).withFeedForward(feedForward)
                .withEnableFOC(true));
    }

    public void setVelocityTorqueFOC(double velocity) {
        final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);
        talon.setControl(request.withVelocity(velocity / velocityConversionFactor));
    }

    // position torque foc
    public void setPositionTorqueFOC(double setpoint) {
        final PositionTorqueCurrentFOC request = new PositionTorqueCurrentFOC(0).withSlot(0);
        talon.setControl(request.withPosition(setpoint).withFeedForward(feedForward));
    }

    // set the motor target position using Motion Magic
    public void setPositionMotionMagic(double position) {
        final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
        talon.setControl(request.withPosition(position).withFeedForward(feedForward).withEnableFOC(true));
    }

    public void setMotionMagicTorqueCurrentFOC(double position){
        final MotionMagicTorqueCurrentFOC request = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
        talon.setControl(request.withPosition(position).withFeedForward(feedForward));
    }

    // set the feedback device of motor - often for using external CANcoders
    public void setFeedbackDevice(int deviceID, FeedbackSensorSourceValue feedbackType) {
        config.Feedback.FeedbackRemoteSensorID = deviceID;
        config.Feedback.FeedbackSensorSource = feedbackType;
        talon.getConfigurator().apply(config);
    }

    // get the current supplied by battery to motor
    public double getSupplyCurrent() {
        return talon.getSupplyCurrent().getValueAsDouble();
    }

    //get stator current used by motor (for voltage control modes)
    public double getStatorCurrent() {
        return talon.getStatorCurrent().getValueAsDouble();
    }

    //get torque current (for torqueCurrent control modes)
    public double getTorqueCurrent(){
        return talon.getTorqueCurrent().getValueAsDouble();
    }

    // get the temperature of the motor
    public double getMotorTemperature() {
        return talon.getDeviceTemp().getValueAsDouble();
    }

    // put motor canID and CANbus name to Smart Dashboard?? Not quite sure what this
    // is really supposed to do
    public void updateSmartdashBoard() {
        SmartDashboard.putNumber(canbusName + " " + deviceID + " motor", 1);
    }

    // Krakens can supposedly play music - don't
    public void playMusic(boolean on) {
        if (on) {
            orchestra.play();
        } else {
            orchestra.pause();
        }
    }

}