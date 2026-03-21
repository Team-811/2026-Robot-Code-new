package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Hold-style command for the main open-loop NEO shooter preset.
 *
 * <p>Because this command never ends by itself, it is meant to be bound with {@code whileTrue(...)}
 * so releasing the button stops the shooter.
 */
public class shooterCommand extends Command {

  private final shooterNeoVortex neoVortex1;

  public shooterCommand(shooterNeoVortex neoVortex1) {
    this.neoVortex1 = neoVortex1;
    addRequirements(neoVortex1);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    neoVortex1.ssssssswirly_whirly();
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
