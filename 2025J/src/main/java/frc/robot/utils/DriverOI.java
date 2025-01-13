package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class DriverOI {

    private static DriverOI instance;
    private Superstructure superstructure;
    private OperatorOI operatorOI;
    private PS4Controller controller;
    private BooleanSupplier algaeMode; // fix this while flushing operator oi pleaseeeee :) :) :) :) :)

    public static DriverOI getInstance() {
        if (instance == null) {
            instance = new DriverOI();
        }
        return instance;
    }

    public DriverOI() {
        superstructure = Superstructure.getInstance();
        operatorOI = OperatorOI.getInstance();
    }

    public void configureController() {
        controller = new PS4Controller(0);

        Trigger touchpadButton = new JoystickButton(controller, PS4Controller.Button.kTouchpad.value);
        touchpadButton.onTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.STOW)));

        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        // L1Bumper.whileTrue(new InstantCommand(() -> superstructure.requestState(SuperstructureState.CORAL_GROUND_INTAKE)));
        L1Bumper.whileTrue(new ConditionalCommand(new InstantCommand(() -> superstructure.requestState(SuperstructureState.ALGAE_GROUND_INTAKE)),
                            new InstantCommand(() -> superstructure.requestState(SuperstructureState.CORAL_GROUND_INTAKE)),
                            algaeMode)); 
    }
}
