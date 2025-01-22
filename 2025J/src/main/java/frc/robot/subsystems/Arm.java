package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;
import frc.robot.utils.LiveData;
import frc.robot.utils.RobotMap;
import frc.robot.utils.TunableConstant;
import frc.robot.utils.Constants.ArmConstants;
import frc.robot.utils.Constants.ElevatorConstants;

public class Arm extends SubsystemBase{

    private static Arm arm;
    private Kraken armMotor;
    private CANcoder armCANcoder;
    private TunableConstant kP, kS, kV, kI, kD, kFF, kA, 
        kArmMaxCruiseVelocity, kArmMaxCruiseAcceleration, kArmMaxCruiseJerk, kArmReverseTorqueCurrentLimit, kArmForwardTorqueCurrentLimit,
        L1Setpoint, L2Setpoint, L3Setpoint, L4Setpoint, HPIntakeSetpoint, stowSetpoint, bargeSetpoint, algaeL1Setpoint, algaeL2Setpoint, processorSetpoint;

    private LiveData armAngle, armSetpoint, motorTemp, motorCurrent;

    public Arm() {
        armCANcoder = new CANcoder(RobotMap.ARM_CANCODER_ID, RobotMap.CANIVORE_NAME);
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        config.MagnetSensor.MagnetOffset = ArmConstants.kArmMagnetOffset;
        armCANcoder.getConfigurator().apply(config);

        armMotor = new Kraken(RobotMap.ARM_MOTOR_ID, RobotMap.CANIVORE_NAME);

        armMotor.setInverted(false);
        armMotor.setSupplyCurrentLimit(ArmConstants.kArmCurrentLimit);
        armMotor.setForwardTorqueCurrentLimit(ArmConstants.kArmForwardTorqueCurrentLimit);
        armMotor.setReverseTorqueCurrentLimit(ArmConstants.kArmReverseTorqueCurrentLimit);

        armMotor.setBrake();

        armMotor.setEncoder(0);
        armMotor.setFeedbackDevice(RobotMap.ARM_CANCODER_ID, FeedbackSensorSourceValue.FusedCANcoder);
        armMotor.setRotorToSensorRatio(ArmConstants.kArmRotorToSensorRatio);
        armMotor.setSensorToMechanismRatio(ArmConstants.kArmSensortoMechanismRatio);

        armMotor.setVelocityPIDValues(ArmConstants.kS, ArmConstants.kV,
                ArmConstants.kA,
                ArmConstants.kP, ArmConstants.kI, ArmConstants.kD,
                ArmConstants.kFF);
        armMotor.setMotionMagicParameters(ArmConstants.kArmMaxCruiseVelocity, ArmConstants.kArmMaxCruiseAcceleration, ArmConstants.kArmMaxCruiseJerk);

        armMotor.setSoftLimits(true, ArmConstants.kArmForwardSoftLimit, ArmConstants.kArmReverseSoftLimit);
        kP = new TunableConstant(ArmConstants.kP, "Arm kP");
        kI = new TunableConstant(ArmConstants.kI, "Arm kI");
        kD = new TunableConstant(ArmConstants.kD, "Arm kD");
        kA = new TunableConstant(ArmConstants.kA, "Arm A"); 
        kS = new TunableConstant(ArmConstants.kS, "Arm kS");
        kV = new TunableConstant(ArmConstants.kV, "Arm kV");
        kFF = new TunableConstant(ArmConstants.kFF, "Arm kFF");
        kArmMaxCruiseVelocity = new TunableConstant(ArmConstants.kArmMaxCruiseVelocity, "Arm kArmMaxCruiseVelocity");
        kArmMaxCruiseAcceleration = new TunableConstant(ArmConstants.kArmMaxCruiseAcceleration, "Arm kArmMaxCruiseAcceleration");
        kArmMaxCruiseJerk = new TunableConstant(ArmConstants.kArmMaxCruiseJerk, "Arm kArmMaxCruiseJerk");
        kArmReverseTorqueCurrentLimit = new TunableConstant(ArmConstants.kArmReverseTorqueCurrentLimit, "Arm kArmReverseTorqueCurrentLimit");
        kArmForwardTorqueCurrentLimit = new TunableConstant(ArmConstants.kArmForwardTorqueCurrentLimit, "Arm kArmForwardTorqueCurrentLimit");
        L1Setpoint = new TunableConstant(ArmConstants.L1Setpoint, "Arm L1Setpoint");
        L2Setpoint = new TunableConstant(ArmConstants.L2Setpoint, "Arm L2Setpoint");
        L3Setpoint = new TunableConstant(ArmConstants.L3Setpoint, "Arm L3Setpoint");
        L4Setpoint = new TunableConstant(ArmConstants.L4Setpoint, "Arm L4Setpoint");
        HPIntakeSetpoint = new TunableConstant(ArmConstants.HPIntakeSetpoint, "Arm HPIntakeSetpoint");
        stowSetpoint = new TunableConstant(ArmConstants.stowSetpoint, "Arm stowSetpoint");
        bargeSetpoint = new TunableConstant(ArmConstants.bargeSetpoint, "Arm bargeSetpoint");
        algaeL1Setpoint = new TunableConstant(ArmConstants.algaeL1Setpoint, "Arm algaeL1Setpoint");
        algaeL2Setpoint = new TunableConstant(ArmConstants.algaeL2Setpoint, "Arm algaeL2Setpoint");
        processorSetpoint = new TunableConstant(ArmConstants.processorSetpoint, "Arm processorSetpoint");

        armSetpoint = new LiveData(stowSetpoint.get(), "Arm Current Setpoint"); 
        armAngle = new LiveData(getAbsoluteCANcoderPosition(), "Arm Current Angle"); 

        motorTemp = new LiveData(armMotor.getMotorTemperature(), "Arm Motor Temp"); 
        motorCurrent = new LiveData(armMotor.getSupplyCurrent(), "Arm Motor Current");

        
    }

    public static Arm getInstance(){
        if (arm == null){
            arm = new Arm();
        }
        return arm;
    }

    public double getAbsoluteCANcoderPosition() {
        return armCANcoder.getAbsolutePosition().getValueAsDouble();
    }

    public void setArmPercentOutput(double percentOutput){
        armMotor.setMotor(percentOutput);
    }

    public void setArmPositionVoltage(double position){
        armMotor.setPosition(position);
    }

    public void setArmPositionMotionMagic(double position){
        armMotor.setPositionMotionMagic(position);
    }

    public void setMotionMagicTorqueCurrentFOC(double position){
        armMotor.setMotionMagicTorqueCurrentFOC(position);
    }

    public double getArmAngleDegrees() {
        //TODO: implement this code
        return 0;
    }

    public boolean isAtAngle(double targetAngle) {
        return Math.abs(getArmAngleDegrees() - targetAngle) < ArmConstants.kArmPositionEpsilon;
    }

    @Override
    public void periodic() {
        armMotor.setVelocityPIDValues(kS.get(), kV.get(),
            kA.get(),
            kP.get(), kI.get(), kD.get(),
            kFF.get());

        armMotor.setForwardTorqueCurrentLimit(kArmForwardTorqueCurrentLimit.get());
        armMotor.setReverseTorqueCurrentLimit(kArmReverseTorqueCurrentLimit.get());

        armMotor.setMotionMagicParameters(kArmMaxCruiseVelocity.get(), kArmMaxCruiseAcceleration.get(), kArmMaxCruiseJerk.get());
        armAngle.set(getAbsoluteCANcoderPosition());

        motorTemp.set(armMotor.getMotorTemperature()); 
        motorCurrent.set(armMotor.getSupplyCurrent()); 
    }

    @Override
    public void simulationPeriodic() {
    }

}
