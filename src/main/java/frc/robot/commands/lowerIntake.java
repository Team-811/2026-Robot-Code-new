package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeForNow;

/**
 * Hold-style command that lowers the intake arm.
 */
public class lowerIntake extends Command {

  private final intakeForNow in;

  public lowerIntake(intakeForNow in) {
    this.in = in;
    addRequirements(in);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    in.spin();
  }

  @Override
  public void end(boolean interrupted) {
    in.dontSpin();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
