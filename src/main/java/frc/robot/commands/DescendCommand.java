package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Elevator;

/**
 * Timed descent macro.
 *
 * <p>This is the opposite of the climb macro: open the hook, lower the lift for a fixed duration,
 * then stop everything. It uses the same open-loop, time-based philosophy as {@link ClimbCommand}.
 */
public class DescendCommand extends SequentialCommandGroup {
    public DescendCommand(Elevator elevator) {
        addRequirements(elevator);

        addCommands(
            elevator.runOnce(elevator::hookOpen),
            new WaitCommand(ClimberConstants.hookTimeSeconds),
            elevator.runOnce(elevator::stopHook),
            elevator.runOnce(elevator::liftDown),
            new WaitCommand(ClimberConstants.liftStepSeconds),
            elevator.runOnce(elevator::stopLift)
        );
    }
}
