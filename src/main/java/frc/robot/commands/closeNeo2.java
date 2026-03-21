package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Hold-style command for the second close-range NEO shooter RPM preset.
 *
 * <p>The subsystem exposes both {@code close()} and {@code close2()} so the team can compare two
 * nearby shot presets without rewriting the subsystem each time.
 */
public class closeNeo2 extends Command {
  private final shooterNeoVortex neoVortex1;

  public closeNeo2(shooterNeoVortex neoVortex1) {
    this.neoVortex1 = neoVortex1;
    addRequirements(neoVortex1);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    neoVortex1.close2();
  }

  @Override
  public void end(boolean interrupted) {
    neoVortex1.ssssssswirly_whirly_stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
