package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Hold-style reverse command for the NEO shooter.
 *
 * <p>This now asks the Spark Flex controllers for a negative RPM setpoint instead of a raw percent
 * output, which keeps the Vortex shooter consistent with the rest of the project's velocity-based
 * shooter control.
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
