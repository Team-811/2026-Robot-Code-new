package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Hold-style reverse command for the NEO shooter.
 *
 * <p>This is primarily a recovery/jam-clearing command rather than a scoring command.
 */
public class reverseShooter extends Command {
  private final shooterNeoVortex shooter;

  public reverseShooter(shooterNeoVortex shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.spinTheOtherWay();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.ssssssswirly_whirly_stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
