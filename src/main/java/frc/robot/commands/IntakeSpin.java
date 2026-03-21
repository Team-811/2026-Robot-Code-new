package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Hold-style command that runs the intake roller inward.
 *
 * <p>This is a classic command-based pattern: execute keeps the mechanism running, end stops it, and
 * {@link #isFinished()} returns {@code false} so the button binding controls the lifetime.
 */
public class IntakeSpin extends Command {

  private final Intake spinney;

  public IntakeSpin(Intake spinney) {
    this.spinney = spinney;
    addRequirements(spinney);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    spinney.spin();
  }

  @Override
  public void end(boolean interrupted) {
    spinney.dontspin();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
