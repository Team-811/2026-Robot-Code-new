// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/*
 * File Overview: Defines all robot-wide constants used across subsystems/commands.
 * Features/Details:
 * - OperatorConstants: controller ports, joystick deadbands, drivetrain speed presets.
 * - Hardware IDs: CAN IDs and PCM channels for motors/solenoids used elsewhere.
 * - Tunables: shared slip current limit and elevator deadband.
 * Keep this file logic-free so values can be imported safely from anywhere.
 */
/**
 * Central place for robot-wide constants. Keep this free of behavior/logic so these values can be
 * referenced from anywhere without side effects.
 */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    private OperatorConstants() {}

    // Controller ports
    public static final int kDriverControllerPort  = 0;  // USB port for driver controller
    public static final int kOpControllerPort      = 1;  // USB port for operator/controller 2

    // Shared deadzone for stick axes (override individually if needed later).
    public static final double kDefaultControllerDeadzone = 0.1;                // Default joystick deadband
    public static final double kJoyRightXDeadzone = kDefaultControllerDeadzone; // Deadband for right stick X axis
    public static final double kJoyLeftXDeadzone = kDefaultControllerDeadzone;  // Deadband for left stick X axis
    public static final double kJoyLeftYDeadzone = kDefaultControllerDeadzone;  // Deadband for left stick Y axis

    // Base speed scaler applied to the drivetrain (0-1).
    // USAGE: For SLOW -> Hold right bumper -> slowSpeed (0.5).
    //        For FAST -> Hold left bumper -> fastSpeed (1.0).
    //        For NORMAL: Neither bumper -> normalSpeed (0.8 default).
    public static final double kSpeed = 1.0;        // Base drivetrain speed 'scale' (0-1)

    // Driver speed scaling presets
    public static final double fastSpeed     = 1.0; // Driver fast speed preset
    public static final double slowSpeed     = 0.5; // Driver slow speed preset
    public static final double normalSpeed   = 0.8; // Driver normal speed preset

    // Hardware IDs and limits
    public static final double kSlipCurrent  = 120; // Current threshold for slip protection
    public static final int neoId            = 18;  // CAN ID for NEO motor controller
    public static final int elKrakenId       = 23;  // CAN ID for elevator Kraken motor
    public static final int fDoubSolC1       = 2;   // PCM channel for front double solenoid forward
    public static final int rDoubSolC1       = 3;   // PCM channel for rear double solenoid forward
    public static final int fDoubsolCT       = 6;   // PCM channel for front double solenoid reverse
    public static final int rDoubSolCT       = 7;   // PCM channel for rear double solenoid reverse
    public static final int fDoubSolA        = 0;   // PCM module address for front double solenoid
    public static final int rDoubSolA        = 1;   // PCM module address for rear double solenoid
    public static final int cArmId           = 24;  // CAN ID for arm motor controller
    public static final int climbId          = 25;  // CAN ID for climber motor controller
    public static final double kElDeadBand   = 3;   // Elevator deadband to ignore small commands

    // Shooter CAN IDs (two motors driving the shooter wheels).
    public static final int shooterTopId     = 26;
    public static final int shooterBottomId  = 27;
  }
}
