package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Elevator;

/**
 * Climb sequence:
 * 1) Close hook
 * 2) Lift up
 * 3) Open hook for next rung
 * 4) Close hook on next rung
 * 5) Lift again
 * Uses timed steps for simplicity (no sensors/limits). Tune durations in ClimberConstants for your rig.
 * Rationale: without encoders/limit switches on the lift/hook, timed open-loop steps are the safest
 * quick solution. Keep speeds conservative and test on a practice robot before competition use.
 */
public class ClimbCommand extends SequentialCommandGroup {
    public ClimbCommand(Elevator elevator) {
        addRequirements(elevator);

        addCommands(
            // Close hook
            elevator.runOnce(elevator::hookClose),
            new WaitCommand(ClimberConstants.hookTimeSeconds),
            elevator.runOnce(elevator::stopHook),

            // Lift first step
            elevator.runOnce(elevator::liftUp),
            new WaitCommand(ClimberConstants.liftStepSeconds),
            elevator.runOnce(elevator::stopLift),
            new WaitCommand(ClimberConstants.settleTimeSeconds),

            // Open then close hook on next rung
            elevator.runOnce(elevator::hookOpen),
            new WaitCommand(ClimberConstants.hookTimeSeconds),
            elevator.runOnce(elevator::hookClose),
            new WaitCommand(ClimberConstants.hookTimeSeconds),
            elevator.runOnce(elevator::stopHook),

            // Lift second step
            elevator.runOnce(elevator::liftUp),
            new WaitCommand(ClimberConstants.liftStepSeconds),
            elevator.runOnce(elevator::stopLift)
        );
    }
}
