package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeForNow;

/**
 * Hold-style command that raises the intake arm.
 *
 * <p>The subsystem method name is {@code spinTheOtherWay()} because the motor is currently controlled
 * as a simple bi-directional open-loop output rather than a named position loop.
 */
public class raiseIntake extends Command {
  private final intakeForNow in;

  public raiseIntake(intakeForNow in) {
    this.in = in;
    addRequirements(in);
  }

  @Override
  public void initialize() {
    in.spinTheOtherWay();
  }

  @Override
  public void execute() {
    in.spinTheOtherWay();
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
