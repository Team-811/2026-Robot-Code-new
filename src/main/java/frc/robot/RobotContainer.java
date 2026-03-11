// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
//
// Driver (controller 0):
 //   - Left stick translation, right stick X rotation; velocity slew-limited, speed modes: slow 35%, normal 70%, fast 100%.
//   - Y toggles field-centric vs robot-centric drive.
//   - LB/RB/X latch slow/fast/normal speed modes; 
//   - B (hold) runs FaceAprilTag to aim at tags 2,3,4,10.
//   - Start or Back reseeds the field-centric heading to the current gyro angle.
//
// Operator (controller 1):
//   - B: spin intake; 
//   - A: run indexer; 
//   - Y: fast shooter spin (closeNeo2); 
//   - LB/RB: raise/lower intake pivot motor.
//   - LT: slow shooter spin; 
//   - X: medium shooter spin; 
//   - RT: Limelight-based Falcon shooter; 
//   - Start: reverse shooter.
//   - POV Up: run climb sequence (hook, lift, hook next rung, lift); 
//   - POV Down: descend sequence (open hook, lower).
//
// Autonomous chooser:
//   - SmartDashboard key "autoChooser" exposes "leftAuto" and "leftDriveAuto" PathPlanner sequences.
//
// Notes:
//   - Default command is CTRE field-centric drive with slew rate limiting; speed modes scale the cap.
//   - Drivetrain telemetry is streamed via Telemetry; LimelightShooter pushes updates in its periodic.
package frc.robot;

/*
 * File Overview: Central wiring hub for subsystems, commands, and driver controls.
 * Features/Details:
 * - Instantiates the CTRE swerve drivetrain, shooter/indexer/intake subsystems, Limelight, and telemetry logger.
 * - Wires driver controls for smooth field- or robot-centric driving, speed modes, and quick AprilTag snap-to-heading.
 * - Wires operator controls for intake/indexer/shooter motors with distinct speed presets per button.
 * - Registers a PathPlanner-backed autonomous chooser on the dashboard.
 * - Seeds field-centric heading at startup and streams drivetrain telemetry continuously.
 */
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FaceAprilTag;
import frc.robot.commands.shooterCommand;
import frc.robot.commands.shooterLime;
import frc.robot.commands.IndexSpin;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.closeNeo2;
import frc.robot.commands.closeShooter;
import frc.robot.commands.leftAuto;
import frc.robot.commands.leftDriveAuto;
import frc.robot.commands.lowerIntake;
import frc.robot.commands.raiseIntake;
import frc.robot.commands.reverseShooter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intakeForNow;       
import frc.robot.subsystems.shooterNeoVortex;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CANdleLED;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DescendCommand;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import static edu.wpi.first.units.Units.*;
public class RobotContainer {
  // TunerConstants encapsulates the drivetrain's measured free-speed; scaling happens in the drive request.
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // Cap rotation to ~470 deg/s for driver comfort; raise if you want snappier snap-turns.
  private final double MaxAngularRate = RotationsPerSecond.of(1.3).in(RadiansPerSecond);
  private double speedModeScalar = OperatorConstants.normalSpeed;
  private String speedModeLabel = "normal";

  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();  
  private final intakeForNow intakeArm = new intakeForNow();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    

  /** Deadband applied to joystick inputs before slew limiting; 0.05–0.1 removes stick drift without hiding fine aim. */
  private static final double DEADBAND = 0.08;
  // Slew limiters in real units to smooth accel/decel instead of stick space.
  private final SlewRateLimiter velLimiterY = new SlewRateLimiter(3.0);
  private final SlewRateLimiter velLimiterX = new SlewRateLimiter(3.0);
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(6.0);
  private boolean fieldRelative = true;

  private final Shooter shooter = new Shooter();
  private final shooterNeoVortex shooterN = new shooterNeoVortex();
  private final Elevator elevator = new Elevator();


  private final LimelightShooter limeShooter = new LimelightShooter(); // primary LL4 (scoring/AprilTag aim)
  private final CANdleLED candle = new CANdleLED(LimelightShooter::hasTarget);


  // Driver controls (single Xbox assumed for drivetrain + vision assist).
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final CommandXboxController c_OperatorController = new CommandXboxController(OperatorConstants.kOpControllerPort);

  private final SendableChooser<String> autoChooser;

  /** Constructs the robot container: builds subsystems, seeds heading, binds controls, and prepares autos. */
  public RobotContainer() {
    // Seed heading at startup so field-centric drive has a sane reference.
    drivetrain.seedFieldCentric();
    configureBindings();
    publishStaticTelemetry();

    // Expose autos on SmartDashboard as simple string keys (mapped to commands later).
    autoChooser = new SendableChooser<String>();
    autoChooser.addOption("leftAuto", "leftAuto");
    autoChooser.addOption("leftDriveAuto", "leftDriveAuto");
    SmartDashboard.putData("autoChooser",autoChooser);

  }
  /**
   * Wire driver controls to commands. Sets the default drive command and bindings for vision and SysId.
   */
  private void configureBindings() {
    

    // Default: smooth field-centric drive with conservative caps so practice driving stays tame.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
            double xInput = -MathUtil.applyDeadband(m_driverController.getLeftY(), DEADBAND);
            double yInput = -MathUtil.applyDeadband(m_driverController.getLeftX(), DEADBAND);
            double rotInput = -MathUtil.applyDeadband(m_driverController.getRightX(), DEADBAND);

            double targetVx = xInput * MaxSpeed * speedModeScalar;
            double targetVy = yInput * MaxSpeed * speedModeScalar;
            double targetOmega = rotInput * MaxAngularRate * speedModeScalar;

            double vx = velLimiterX.calculate(targetVx);
            double vy = velLimiterY.calculate(targetVy);
            double omega = omegaLimiter.calculate(targetOmega);

            return buildDriveRequest(vx, vy, omega);
        })
    );

    // Toggle field-/robot-centric driving on the driver Y button.
    m_driverController.y().onTrue(
        Commands.runOnce(() -> {
          fieldRelative = !fieldRelative;
          SmartDashboard.putBoolean("Drive/FieldRelative", fieldRelative);
        }, drivetrain).ignoringDisable(true)
    );
    // Driver speed modes: LB = slow, RB = fast, X = normal.
    m_driverController.leftBumper().onTrue(
        Commands.runOnce(() -> setSpeedMode(OperatorConstants.slowSpeed, "slow"), drivetrain).ignoringDisable(true)
    );
    m_driverController.rightBumper().onTrue(
        Commands.runOnce(() -> setSpeedMode(OperatorConstants.fastSpeed, "fast"), drivetrain).ignoringDisable(true)
    );
    m_driverController.x().onTrue(
        Commands.runOnce(() -> setSpeedMode(OperatorConstants.normalSpeed, "normal"), drivetrain).ignoringDisable(true)
    );

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );
    // CANdle auto-updates via subsystem periodic; no button bindings required.

    //-- SHOOTER VISION -- Vision-assisted align/target commands.
    // Run face-to-tag only while B is held so driver regains control on release.
    m_driverController.b().whileTrue(new FaceAprilTag(drivetrain, limeShooter));

    // Heading reseed helpers (both Start and Back give drivers a quick way to fix field-centric zero).
    m_driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    m_driverController.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
             
    // Operator controls: intake/indexer/shooter with distinct presets per button.
    c_OperatorController.b().whileTrue(new IntakeSpin(intake));
    c_OperatorController.a().whileTrue(new IndexSpin(indexer));
    c_OperatorController.y().whileTrue(new closeNeo2(shooterN));
    c_OperatorController.leftBumper().whileTrue(new raiseIntake(intakeArm));
    c_OperatorController.rightBumper().whileTrue(new lowerIntake(intakeArm));
    
    c_OperatorController.leftTrigger().whileTrue(new shooterCommand(shooterN));
    c_OperatorController.x().whileTrue(new closeShooter(shooterN));
    c_OperatorController.rightTrigger().whileTrue(new shooterLime(shooter));
    c_OperatorController.start().whileTrue(new reverseShooter(shooterN));

    // Climber: POV up to run climb sequence, POV down to descend. Toggle cancels/restarts.
    c_OperatorController.povUp().toggleOnTrue(new ClimbCommand(elevator));
    c_OperatorController.povDown().toggleOnTrue(new DescendCommand(elevator));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /** One-time dashboard entries that do not change at runtime. */
  private void publishStaticTelemetry() {
    SmartDashboard.putNumber("Drive/MaxSpeedMps", MaxSpeed);
    SmartDashboard.putNumber("Drive/MaxAngularRateRadPerSec", MaxAngularRate);
    SmartDashboard.putBoolean("Drive/FieldRelative", fieldRelative);
    SmartDashboard.putString("Drive/SpeedMode", speedModeLabel);
  }

  /**
   * Builds the autonomous command selected on the dashboard chooser.
  * Chooser stores string keys; this maps them to concrete command objects.
  * Falls back to PathPlanner "leftAuto" if nothing is selected.
  */
  public Command getAutonomousCommand() {
        String Choice = autoChooser.getSelected();
    Command auto;
    switch (Choice) {
      case "leftAuto":
        auto= new leftAuto(drivetrain, shooterN, shooter, intakeArm, indexer, intake, limeShooter);
        break;
      case "leftDriveAuto":
        auto = new leftDriveAuto(drivetrain, shooterN, shooter, limeShooter, indexer);
        break;
    case "justMove":
      default: 
      auto= new PathPlannerAuto("leftAuto");
        break;
    }
    return auto;

  }

  /** Build either field- or robot-centric drive requests using already-scaled velocities. */
  private SwerveRequest buildDriveRequest(double vx, double vy, double omega) {
    if (fieldRelative) {
      return new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withVelocityX(vx)
          .withVelocityY(vy)
          .withRotationalRate(omega);
    }
    return new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withVelocityX(vx)
        .withVelocityY(vy)
        .withRotationalRate(omega);
  }

  private void setSpeedMode(double scalar, String label) {
    speedModeScalar = MathUtil.clamp(scalar * OperatorConstants.drivetrainSpeedCap, 0.0, 1.0);
    speedModeLabel = label;
    SmartDashboard.putString("Drive/SpeedMode", speedModeLabel);
  }
}
