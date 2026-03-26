package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/** Instant command that switches the robot to the normal driver speed preset. */
public class SetNormalDriveMode extends InstantCommand {
  public SetNormalDriveMode(RobotContainer robotContainer) {
    super(robotContainer::setNormalDriveMode);
  }
}
