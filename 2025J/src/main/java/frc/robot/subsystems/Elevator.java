package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class Elevator extends SubsystemBase {
    private Kraken elevatorMainMotor, elevatorFollowerMotor;

    public Elevator() {
        elevatorMainMotor = new Kraken(RobotMap.ELEVATOR_MAIN_ID, RobotMap.CANIVORE_NAME);
        elevatorFollowerMotor = new Kraken(RobotMap.ELEVATOR_SECONDARY_ID, RobotMap.CANIVORE_NAME);

        elevatorMainMotor.setInverted(false); // TODO: confirm direction
        elevatorFollowerMotor.setFollower(RobotMap.ELEVATOR_MAIN_ID, true);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
