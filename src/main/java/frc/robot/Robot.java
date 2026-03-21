// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Top-level WPILib robot class.
 *
 * <p>In a command-based project, this class stays intentionally small. WPILib calls its lifecycle
 * methods when the robot changes modes, but almost all robot behavior lives elsewhere:
 * {@link RobotContainer} owns the subsystems, button bindings, and autonomous command creation.
 *
 * <p>A useful mental model is:
 * <ul>
 *   <li>{@link Robot} handles mode transitions</li>
 *   <li>{@link RobotContainer} defines what the robot is made of and how operators control it</li>
 *   <li>{@link CommandScheduler} runs the command-based framework every 20 ms</li>
 * </ul>
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  /**
   * Runs once when the robot program starts.
   *
   * <p>This is the right place to construct long-lived robot objects. Building
   * {@link RobotContainer} here causes all subsystems, commands, and chooser entries to exist before
   * the first scheduler loop.
   */
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * Called every 20 ms, no matter which mode the robot is in.
   *
   * <p>This method must keep running the {@link CommandScheduler} or the command-based framework
   * will effectively stop: button bindings will not fire, commands will not execute, and subsystem
   * {@code periodic()} methods will not be called.
   *
   * <p>This project also mirrors the current field match time into {@code FMSInfo/MatchTime} so
   * dashboards or coprocessors can read it from the standard NetworkTables location.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    double time = DriverStation.getMatchTime();
    NetworkTableInstance.getDefault()
        .getTable("FMSInfo")
        .getEntry("MatchTime")
        .setDouble(time);
  }

  /** Called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /**
   * Called once when Autonomous mode begins.
   *
   * <p>The selected autonomous command is requested from {@link RobotContainer} and scheduled if
   * one exists.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  /**
   * Called once when Teleop begins.
   *
   * <p>Any running autonomous command is canceled so the drivers and operators immediately regain
   * control of the robot.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  /**
   * Called once when Test mode begins.
   *
   * <p>All commands are canceled so test starts from a clean state.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
