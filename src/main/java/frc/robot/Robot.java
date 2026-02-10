// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*
 * File Overview: Implements the WPILib TimedRobot lifecycle and hands control to RobotContainer.
 * Features/Details:
 * - Constructs RobotContainer once to wire subsystems, commands, and autos.
 * - Runs CommandScheduler every 20ms in robotPeriodic to service bindings and subsystem periodic.
 * - Manages mode transitions: cancels auto at teleop, clears commands in test, seeds autos from chooser.
 * - Simulation hooks provided (simulationInit/Periodic) if running in sim.
 */
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Implements the TimedRobot lifecycle and delegates command setup/routing to RobotContainer.
 * If you change the name of this class or the package after creating this project, you must also
 * update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  /** Runs once on startup; use this for construction and initialization. */
  public Robot() {
    // Instantiate RobotContainer to wire up subsystems, commands, and the autonomous chooser.
    m_robotContainer = new RobotContainer();
  }

  /**
   * Called every 20 ms in all modes. The scheduler polls inputs, schedules/cancels commands, and
   * runs subsystem periodic hooks; this must run for the command-based framework to operate.
   */
  @Override
  public void robotPeriodic() {
    // Keep the command scheduler alive each loop so bindings and subsystems stay updated.
    CommandScheduler.getInstance().run();
  }

  /** Invoked once when the robot transitions into Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** Starts the currently selected autonomous command from {@link RobotContainer}. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Cancel autonomous before driver control so commands do not fight each other.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Clear out any running commands so test mode starts from a known state.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** Called once when starting up in simulation. */
  @Override
  public void simulationInit() {}

  /** Called periodically while running in simulation. */
  @Override
  public void simulationPeriodic() {}
}
