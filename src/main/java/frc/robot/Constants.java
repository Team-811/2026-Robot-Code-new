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

    // Controller ports (USB order in DriverStation)
    public static final int kDriverControllerPort  = 0;  // Primary/drive
    public static final int kOpControllerPort      = 1;  // Operator/secondary

    // Shared deadzone for stick axes; 0.05-0.15 is common. Keep small to retain fine aiming control.
    public static final double kDefaultControllerDeadzone = 0.1;
    public static final double kJoyRightXDeadzone = kDefaultControllerDeadzone; // Rotation
    public static final double kJoyLeftXDeadzone  = kDefaultControllerDeadzone; // Strafe
    public static final double kJoyLeftYDeadzone  = kDefaultControllerDeadzone; // Forward/back
               
    // Base speed scaler applied to the drivetrain (0-1). Tune down for rookies or tight fields.
    public static final double kSpeed = 1.0;

    // Driver speed scaling presets (multiplied into MaxSpeed/MaxAngularRate).
    // Use 0.8-1.0 for full field, 0.3-0.6 for controlled scoring lanes.
    public static final double fastSpeed     = 1.0; // Full power
    public static final double slowSpeed     = 0.1; // Crawl/precision
    public static final double normalSpeed   = 0.5; // Everyday practice speed

    // Hardware IDs and limits
    public static final double kSlipCurrent  = 120; // Amps where wheels likely slip; typical FRC range 80-150 A
    public static final int neoId            = 18;  // CAN ID for NEO motor controller
    public static final int elKrakenId       = 23;  // CAN ID for elevator Kraken motor
    public static final int fDoubSolC1       = 2;   // PCM channel for front double solenoid forward
    public static final int rDoubSolC1       = 3;   // PCM channel for rear double solenoid forward
    public static final int fDoubsolCT       = 6;   // PCM channel for front double solenoid reverse
    public static final int rDoubSolCT       = 7;   // PCM channel for rear double solenoid reverse
    public static final int fDoubSolA        = 0;   // PCM module address for front double solenoid
    public static final int rDoubSolA        = 1;   // PCM module address for rear double solenoid
    public static final int cArmId           = 24;  // CAN ID for arm motor controller
    public static final int climbId          = 30;  // CAN ID for climber motor controller
    public static final double kElDeadBand   = 3;   // Elevator deadband to ignore small commands

    // Shooter CAN IDs (two motors driving the shooter wheels).
    public static final int shooterTopId     = 26;
    public static final int shooterBottomId  = 27;
  }

  /** Hardware mapping and tunables for the CANdle LED controller. */
  public static final class CANdleConstants {
    private CANdleConstants() {}

    // Set this to the CAN ID configured for your CANdle in Phoenix Tuner.
    public static final int candleCanId = 30;
    // Number of LEDs to drive. For a bare CANdle with only the built-in RGBs, this is 8.
    public static final int ledCount = 8;
    // Limelight pipeline index this indicator should watch (0 = AprilTag pipeline in this project).
    public static final int pipelineIndex = 0;
  }

  /**
   * Hardware IDs and speeds for the climber (dual NEO lift + hook motor).
   * Rationale:
   * - IDs are split so left/right lifts can be wired independently; keep them unique on the CAN bus.
   * - Speeds are open-loop percents to keep sequencing simple; tune cautiously on a practice rig.
   * - Timings are coarse delays to coordinate hook/lift without sensors; expect to tune these on a real robot.
   */
  public static final class ClimberConstants {
    private ClimberConstants() {}

    // CAN IDs (set to your wiring; ensure they are unique to avoid CAN conflicts)
    public static final int liftLeftNeoId  = 31;
    public static final int liftRightNeoId = 32;
    public static final int hookMotorId    = 33; // Johnson gearmotor controller

    // Speeds (percent output 0-1). Keep conservative to avoid shock loads; increase only after testing.
    public static final double liftUpSpeed     = 0.7;
    public static final double liftDownSpeed   = -0.4;
    public static final double hookCloseSpeed  = 0.5;
    public static final double hookOpenSpeed   = -0.5;

    // Timed sequence durations (seconds). These are dead-reckoned delays; tune on a practice rig for your geometry.
    public static final double hookTimeSeconds     = 0.6;
    public static final double liftStepSeconds     = 2.0;
    public static final double settleTimeSeconds   = 0.2;
  }
}
