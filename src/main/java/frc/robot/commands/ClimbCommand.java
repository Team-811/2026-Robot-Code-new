package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Elevator;

/**
 * Timed climb macro.
 *
 * <p>This command uses {@link SequentialCommandGroup} so the climb reads like a checklist:
 * close hook, lift, reposition hook, then lift again.
 *
 * <p>Because the climber subsystem does not currently use sensors or position control, every stage is
 * open-loop and time-based. That makes the sequence easy to read and easy to tune, but it also means
 * the constants in {@link frc.robot.Constants.ClimberConstants} matter a lot.
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
