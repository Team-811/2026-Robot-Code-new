package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Hold-style command for the Limelight-assisted Talon shooter.
 *
 * <p>While this command is scheduled, it repeatedly asks the {@link Shooter} subsystem to recalculate
 * distance and update the requested wheel speed. It never finishes on its own, so it is intended for
 * {@code whileTrue(...)} bindings or for use inside a command group that provides a timeout/deadline.
 */
public class shooterLime extends Command {

  private final Shooter shooter;

  public shooterLime(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.runShooterWithLimelight();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
