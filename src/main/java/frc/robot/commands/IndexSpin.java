package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

/**
 * Hold-style command that feeds a note through the indexer toward the shooter.
 */
public class IndexSpin extends Command {

  private final Indexer spinney;

  public IndexSpin(Indexer spinney) {
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
