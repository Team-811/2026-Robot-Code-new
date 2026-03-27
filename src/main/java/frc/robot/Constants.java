// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Central place for robot-wide constants.
 *
 * <p>The main rule for a constants file is that it should stay free of robot behavior. Subsystems
 * and commands can safely import values from here because reading a constant should never create
 * hardware objects, talk to the network, or change state.
 *
 * <p>In this project the nested classes are grouped by purpose:
 * <ul>
 *   <li>{@link OperatorConstants}: controller ports and driver tuning values</li>
 *   <li>{@link CANdleConstants}: LED-related hardware mapping</li>
 *   <li>{@link ClimberConstants}: IDs, open-loop speeds, and timed-step lengths for the climber</li>
 * </ul>
 */
public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    private OperatorConstants() {}

    // USB controller ports as shown in the Driver Station.
    public static final int kDriverControllerPort  = 0;  // Primary/drive
    public static final int kOpControllerPort      = 1;  // Operator/secondary

    // Shared joystick deadzones. These remove small stick drift while still allowing fine control.
    public static final double kDefaultControllerDeadzone = 0.1;
    public static final double kJoyRightXDeadzone = kDefaultControllerDeadzone; // Rotation
    public static final double kJoyLeftXDeadzone  = kDefaultControllerDeadzone; // Strafe
    public static final double kJoyLeftYDeadzone  = kDefaultControllerDeadzone; // Forward/back
               
    // Global drivetrain cap applied on top of the selected speed mode.
    // Set to 1.0 so the speed-mode values below map directly to on-robot feel during tuning.
    public static final double drivetrainSpeedCap = 1.0;

    // Driver speed mode presets.
    // Translation and rotation are split because drivers often want fine approach speed while still
    // keeping enough turn authority to line up quickly.
    public static final double fastDriveSpeed   = 0.55; //.90
    public static final double fastTurnSpeed    = 0.75; //1.00
    public static final double slowDriveSpeed   = 0.35; //.35
    public static final double slowTurnSpeed    = 0.55; //.55
    public static final double normalDriveSpeed = 0.65; //.65
    public static final double normalTurnSpeed  = 0.75; //.75

    // intake arm Kraken ID
    public static final int cArmId           = 56;  // CAN ID for intake arm Kraken X60 / TalonFX controller on the roboRIO CAN bus

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
