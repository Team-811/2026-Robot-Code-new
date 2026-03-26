package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/** Instant command that switches the robot to the slow driver speed preset. */
public class SetSlowDriveMode extends InstantCommand {
  public SetSlowDriveMode(RobotContainer robotContainer) {
    super(robotContainer::setSlowDriveMode);
  }
}
