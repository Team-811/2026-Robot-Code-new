package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Elevator;

/**
 * Simple descent: open hook, lower lift for a fixed time, then stop.
 * Rationale: still open-loop/timed due to lack of sensors; keep speeds/durations conservative to avoid drops.
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
