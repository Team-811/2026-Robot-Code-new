package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Hold-style command for one close-range NEO shooter preset.
 *
 * <p>This command continuously applies the {@link shooterNeoVortex#close()} preset until the button
 * is released or another command interrupts it.
 */
public class closeShooter extends Command {
  private final shooterNeoVortex neoVortex1;

  public closeShooter(shooterNeoVortex neoVortex1) {
    this.neoVortex1 = neoVortex1;
    addRequirements(neoVortex1);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    neoVortex1.close();
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
