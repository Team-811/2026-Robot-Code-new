// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
// Controller Guide - Driver Xbox
//   Left stick: strafe (field-centric X/Y)
//   Right stick X: rotate; deadbands 0.1
//
// Speed modes (Drive/SpeedMode on dashboard):
//   Right bumper = Slow (0.5)
//   Left bumper  = Fast (1.0)
//   Neither      = Normal (0.8)
//
// LimelightShooter (hold-to-run):
//   Right trigger: CenterToTagOneMeter (drive to ~1 m, center X and yaw)
//   Left trigger:  AprilTagAim (drive/strafe/yaw to ~0.3048 m, slow near goal)
//   Pipeline/LEDs must be set to AprilTag with LEDs on; no target -> zero output.
//
// Start button: reseed field-centric heading.
// Start + Y (held): SysId quasistatic forward on drivetrain.
// Start + X (held): SysId quasistatic reverse on drivetrain.
// 
// Notes:
// Default command is field-centric drive; MaxSpeed is scaled by kSpeed=1.0.
// SysId bindings require robot in a safe state; they override normal driving while held.
// LimelightShooter: Targeting is enabled/disabled inside the aim commands; 
//This file is loacted C:\Users\Team 811\FRC\2025-Robot-Code-new\2025-Robot-Code-new\src\main\java\frc\robot
//            Ensure the tag pipeline is active and LEDs on when using triggers.
package frc.robot;

/*
 * File Overview: Central wiring hub for subsystems, commands, and driver controls.
 * Features/Details:
 * - Creates drivetrain (CTRE swerve), Limelight, telemetry logger, and autonomous chooser.
 * - Defines driver Xbox bindings: field-centric default drive, speed modes via bumpers, vision assists on triggers, SysId on start+X/Y.
 * - Applies joystick deadbands/slew rate limiting and speed scaling for smooth control.
 * - Publishes driver-facing telemetry (speed mode/scale, joystick values, pose, velocities) to SmartDashboard.
 * - Seeds field-centric heading at startup and registers drivetrain telemetry streaming.
 */
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FaceAprilTag;
import frc.robot.commands.ShootToAprilTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.LimelightClimber;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.units.Units.*;
public class RobotContainer {

  /** Maximum translational speed (m/s) scaled by operator speed factor. */
  // Base speed scaling constants for the swerve (meters/sec and radians/sec).
  private final double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * OperatorConstants.kSpeed;
  /** Maximum rotational speed (rad/s) for driver rotation input. */
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  // Swerve request object reused by driver bindings.
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Slew limiters tame acceleration in each axis/rotation to keep the robot smooth.
  private final SlewRateLimiter slewLimY = new SlewRateLimiter(2.0);
  private final SlewRateLimiter slewLimX = new SlewRateLimiter(2.0);
  private final SlewRateLimiter slewLimRote = new SlewRateLimiter(1.0);

  private final LimelightShooter limeShooter = new LimelightShooter(); // primary LL (scoring/AprilTag aim)
  private final LimelightClimber limeClimber = new LimelightClimber(); // secondary LL4 for climber/stage
  private final Shooter shooter = new Shooter();

  // Cache last-published driver telemetry to avoid NetworkTables spam.
  private String lastMode;
  private Double lastScale;
  private boolean lastRightBumper;
  private boolean lastLeftBumper;

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Driver controls (single Xbox assumed for drivetrain + vision assist).
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;
  private boolean sysIdActive = false;

  /**
   * Constructs the robot container: builds subsystems, seeds heading, binds controls, and prepares autos.
   */
  public RobotContainer() {
    // Seed heading at startup so field-centric drive has a sane reference.
    drivetrain.seedFieldCentric();
    configureBindings();
    publishStaticTelemetry();

    // Build a PathPlanner-backed autonomous chooser and expose it to SmartDashboard.
    SendableChooser<Command> chooser;
    try {
      chooser = AutoBuilder.buildAutoChooser();
      // Register available autos; "midL4x1" is treated as optional.
      chooser.setDefaultOption("Do Nothing", new InstantCommand());
      chooser.addOption("Ex Auto", new PathPlannerAuto("Ex Auto"));
      chooser.addOption("midL4x1 (if present)", new PathPlannerAuto("midL4x1"));
    } catch (Exception ex) {
      // Fall back to a safe chooser if PathPlanner assets are missing.
      chooser = new SendableChooser<>();
      chooser.setDefaultOption("Do Nothing", new InstantCommand());
      SmartDashboard.putString("Mode/autoChooser/Error", "PathPlanner chooser failed: " + ex.getMessage());
    }
    autoChooser = chooser;
    // Expose chooser so drivers can pick autonomous in the dashboard.
    SmartDashboard.putData("Mode/autoChooser", autoChooser);
  }

  /**
   * Wire driver controls to commands. Sets the default drive command and bindings for vision and SysId.
   */
  private void configureBindings() {
    // Default command: field-centric drive with slew-limited joystick input.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                {
                  double scale = speedScale(); // cache per loop to avoid inconsistent scaling and extra NT writes
                  return drive.withVelocityX(slewLimY.calculate(-joyLeftY()) * MaxSpeed * scale)
                      .withVelocityY(slewLimX.calculate(joyLeftX()) * MaxSpeed * scale)
                      .withRotationalRate(slewLimRote.calculate(-joyRightX()) * MaxAngularRate * scale);
                }));

    //-- SHOOTER VISION -- Vision-assisted align/target commands.
    // Run face-to-tag only while B is held so driver regains control on release.
    driverController.b().whileTrue(new FaceAprilTag(drivetrain, limeShooter));
    // Map right trigger to distance-based shooter feed using Limelight range.
    driverController.rightTrigger().whileTrue(new ShootToAprilTag(shooter, limeShooter));
    
    //-- CLIMBER VISION -- Vision-assisted align/target commands.
    //driverController.y().whileTrue(new FaceTowerClimber(drivetrain, limeClimber));

    // SysId bindings to characterize drivetrain when requested.
    driverController.start().and(driverController.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward).finallyDo((interrupted) -> sysIdActive = false))
        .onTrue(new InstantCommand(() -> sysIdActive = true));
    driverController.start().and(driverController.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse).finallyDo((interrupted) -> sysIdActive = false))
        .onTrue(new InstantCommand(() -> sysIdActive = true));
    driverController.start()
        .and(driverController.y().negate())
        .and(driverController.x().negate())
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
    // Push live drivetrain telemetry to the log so you can monitor speeds, states, and odometry.
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Driver right stick X with deadband applied.
   * @return rotation input in -1..1, zeroed inside deadband
   */
  public double joyRightX() {
    double rightX = driverController.getRightX();
    if (Math.abs(rightX) > OperatorConstants.kJoyRightXDeadzone) {
      return rightX;
    }
    return 0;
  }

  /**
   * Driver left stick X with deadband applied.
   * @return strafe input in -1..1, zeroed inside deadband
   */
  public double joyLeftX() {
    double leftX = driverController.getLeftX();
    if (Math.abs(leftX) > OperatorConstants.kJoyLeftXDeadzone) {
      return leftX;
    }
    return 0;
  }

  /**
   * Driver left stick Y with deadband applied.
   * @return forward/back input in -1..1, zeroed inside deadband
   */
  public double joyLeftY() {
    double leftY = driverController.getLeftY();
    if (Math.abs(leftY) > OperatorConstants.kJoyLeftYDeadzone) {
      return leftY;
    }
    return 0;
  }


  // Variable speed scaling based on bumper state (fast/slow/normal) to tame driver inputs.
  /**
   * Computes current speed scale based on bumper state.
   * @return scalar multiplier applied to translational/rotational commands
   */
  public double speedScale() {
    String mode = "Normal";
    double scale = Constants.OperatorConstants.normalSpeed;
    boolean rightPressed = driverController.rightBumper().getAsBoolean();
    boolean leftPressed = driverController.leftBumper().getAsBoolean();
    if (rightPressed) {
      mode = "Slow";
      scale = Constants.OperatorConstants.slowSpeed;
    }
    if (leftPressed) {
      mode = "Fast";
      scale = Constants.OperatorConstants.fastSpeed;
    }
    boolean modeChanged = modeChanged(rightPressed, leftPressed, scale);
    // Debug telemetry: surface current speed mode/scale to the dashboard.
    pushDriverTelemetry(mode, scale, modeChanged);
    lastMode = mode;
    lastScale = scale;
    lastRightBumper = rightPressed;
    lastLeftBumper = leftPressed;
    return scale;
  }

  /**
   * Detects if speed mode changed (used to gate telemetry updates).
   * @return true if bumper state or scale differ from last check
   */
  private boolean modeChanged(boolean rightPressed, boolean leftPressed, double scale) {
    if (lastMode == null || lastScale == null) {
      return true;
    }
    return rightPressed != lastRightBumper || leftPressed != lastLeftBumper || scale != lastScale;
  }

  /** One-time dashboard entries that do not change at runtime. */
  private void publishStaticTelemetry() {
    SmartDashboard.putNumber("Drive/MaxSpeedMps", MaxSpeed);
    SmartDashboard.putNumber("Drive/MaxAngularRateRadPerSec", MaxAngularRate);
  }

  /** Live driver-focused telemetry for quick debugging and mode awareness. */
   private void pushDriverTelemetry(String mode, double scale, boolean publishModeScale) {
    if (publishModeScale) {
      SmartDashboard.putString("Drive/SpeedMode", mode);
      SmartDashboard.putNumber("Drive/SpeedScale", scale);
    }
    SmartDashboard.putNumber("Drive/RotationScale", scale);
    SmartDashboard.putBoolean("SysId/Running", sysIdActive);
    SmartDashboard.putNumber("Joystick/LeftX", joyLeftX());
    SmartDashboard.putNumber("Joystick/LeftY", joyLeftY());
    SmartDashboard.putNumber("Joystick/RightX", joyRightX());

    var state = drivetrain.getState();
    if (state != null) {
      SmartDashboard.putNumber("Drive/PoseX", state.Pose.getX());
      SmartDashboard.putNumber("Drive/PoseY", state.Pose.getY());
      SmartDashboard.putNumber("Drive/HeadingDeg", state.Pose.getRotation().getDegrees());
      SmartDashboard.putNumber("Drive/MeasuredVx", state.Speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Drive/MeasuredVy", state.Speeds.vyMetersPerSecond);
      SmartDashboard.putNumber("Drive/MeasuredOmega", state.Speeds.omegaRadiansPerSecond);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selected = autoChooser.getSelected();
    return selected != null ? selected : new InstantCommand();
  }
}
