// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DescendCommand;
import frc.robot.commands.FaceAprilTag;
import frc.robot.commands.IndexSpin;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.StartAtHubAuto;
import frc.robot.commands.closeNeo2;
import frc.robot.commands.closeShooter;
import frc.robot.commands.fromRightBumpAuto;
import frc.robot.commands.goToMidAuto;
import frc.robot.commands.justShootAuto;
import frc.robot.commands.leftAuto;
import frc.robot.commands.leftDriveAuto;
import frc.robot.commands.lowerIntake;
import frc.robot.commands.raiseIntake;
import frc.robot.commands.reverseShooter;
import frc.robot.commands.shooterCommand;
import frc.robot.commands.shooterLime;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intakeForNow;
import frc.robot.subsystems.shooterNeoVortex;

/**
 * Central wiring hub for the robot.
 *
 * <p>This file is where a new student should look first when trying to understand the project.
 * It answers three practical questions:
 * <ol>
 *   <li>What subsystems exist on the robot?</li>
 *   <li>What does each controller button do?</li>
 *   <li>Which autonomous routine runs when auto starts?</li>
 * </ol>
 *
 * <p>Current operator map:
 * <ul>
 *   <li>Driver controller (USB 0): left stick translates, right stick rotates, B holds
 *       AprilTag-facing assist, Start/Back reseed field-centric heading.</li>
 *   <li>Operator controller (USB 1): B runs the intake roller, A runs the indexer,
 *       Y/X/LT run preset NEO shooter speeds, RT runs the Limelight-based Talon shooter,
 *       LB/RB move the intake arm, Start reverses the NEO shooter, POV up/down toggle
 *       climb macros.</li>
 * </ul>
 *
 * <p>Important accuracy note: this class still stores a {@code fieldRelative} flag and
 * {@code speedModeScalar}, and the helper method {@link #buildDriveRequest(double, double, double)}
 * uses them correctly. However, the currently active default drive command below directly creates a
 * {@link SwerveRequest.FieldCentric} request with fixed scaling, so the Y/LB/RB/X buttons currently
 * update dashboard state more than robot behavior. That is useful to know when reading the code.
 */
public class RobotContainer {
  // Phoenix Tuner X measures the drivetrain's free speed; we use that as the base scale for joystick commands.
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // 2.5 rotations/second base angular rate. The active default command later scales this by 0.2.
  private final double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond);
  private double speedModeScalar = OperatorConstants.normalSpeed;
  private String speedModeLabel = "normal";

  // Game piece path through the robot: intake roller -> indexer -> shooter.
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  // Despite the temporary class name, this subsystem is the intake arm / pivot motor.
  private final intakeForNow intakeArm = new intakeForNow();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  // Generated CTRE drivetrain wrapper. It owns the swerve modules, odometry, and PathPlanner hooks.
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /** Deadband applied before slew limiting so tiny stick drift does not move the robot. */
  private static final double DEADBAND = 0.08;
  // Slew limiters smooth sudden joystick changes so the robot feels less twitchy and breaks traction less often.
  private final SlewRateLimiter slewLimY = new SlewRateLimiter(3);
  private final SlewRateLimiter slewLimX = new SlewRateLimiter(3);
  private final SlewRateLimiter slewLimRote = new SlewRateLimiter(6);
  private boolean fieldRelative = true;

  // Two shooter implementations are present:
  // - Shooter: TalonFX velocity loop with Limelight distance-based RPM lookup.
  // - shooterNeoVortex: SparkFlex velocity presets expressed directly in RPM.
  private final Shooter shooter = new Shooter();
  private final shooterNeoVortex shooterN = new shooterNeoVortex();
  private final Elevator elevator = new Elevator();

  private final LimelightShooter limeShooter = new LimelightShooter();

  // Driver operates drivetrain and alignment assist.
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // Operator runs the scoring mechanisms and climb.
  private final CommandXboxController c =
      new CommandXboxController(OperatorConstants.kOpControllerPort);

  private final SendableChooser<String> autoChooser;

  /**
   * Build the robot container once at startup.
   *
   * <p>The constructor seeds the drivetrain heading, installs all button bindings, publishes
   * dashboard values, and creates the autonomous chooser that {@link Robot} queries later.
   */
  public RobotContainer() {
    // Give field-centric drive an initial forward reference as soon as the code boots.
    drivetrain.seedFieldCentric();
    configureBindings();
    publishStaticTelemetry();

    // SmartDashboard chooser stores simple string keys. getAutonomousCommand() turns those keys into commands.
    autoChooser = new SendableChooser<String>();
    autoChooser.addOption("leftAuto", "leftAuto");
    autoChooser.addOption("leftDriveAuto", "leftDriveAuto");
    autoChooser.addOption("goToMidAuto", "goToMidAuto");
    autoChooser.addOption("fromRightBumpAuto", "fromRightBumpAuto");
    autoChooser.addOption("StartAtHubAuto", "StartAtHubAuto");
    autoChooser.addOption("justShootAuto", "justShootAuto");
    SmartDashboard.putData("autoChooser", autoChooser);
  }

  /**
   * Connect controller buttons to commands and install the drivetrain default command.
   *
   * <p>In the command-based framework, the default command is the background behavior for a
   * subsystem. It runs continuously until another command that requires that subsystem interrupts it.
   */
  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          double x = slewLimX.calculate(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), DEADBAND));
          double y = slewLimY.calculate(
              -MathUtil.applyDeadband(m_driverController.getLeftX(), DEADBAND));
          double rot = slewLimRote.calculate(
              -MathUtil.applyDeadband(m_driverController.getRightX(), DEADBAND));

          // This is the actual live drive request today: always field-centric with fixed scaling.
          return new SwerveRequest.FieldCentric()
              .withDriveRequestType(DriveRequestType.Velocity)
              .withVelocityX(x * MaxSpeed * 0.7)
              .withVelocityY(y * MaxSpeed * 0.7)
              .withRotationalRate(rot * MaxAngularRate * 0.2);
        }));

    // Toggle the remembered field-relative state and dashboard indicator.
    // The helper method below honors this flag, but the current live default command above does not.
    m_driverController.y().onTrue(
        Commands.runOnce(() -> {
          fieldRelative = !fieldRelative;
          SmartDashboard.putBoolean("Drive/FieldRelative", fieldRelative);
        }, drivetrain).ignoringDisable(true));

    // These buttons update the stored speed mode and dashboard label.
    // They are useful state for the helper method, but the active default command currently uses fixed output scales.
    m_driverController.leftBumper().onTrue(
        Commands.runOnce(() -> setSpeedMode(OperatorConstants.slowSpeed, "slow"), drivetrain)
            .ignoringDisable(true));
    m_driverController.rightBumper().onTrue(
        Commands.runOnce(() -> setSpeedMode(OperatorConstants.fastSpeed, "fast"), drivetrain)
            .ignoringDisable(true));
    m_driverController.x().onTrue(
        Commands.runOnce(() -> setSpeedMode(OperatorConstants.normalSpeed, "normal"), drivetrain)
            .ignoringDisable(true));

    // While disabled, explicitly idle the swerve request so motors are not being driven by stale commands.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // Hold B to temporarily let the Limelight rotate the robot toward the selected AprilTag.
    m_driverController.b().whileTrue(new FaceAprilTag(drivetrain, limeShooter));

    // These two buttons both reseed the field-centric heading so drivers can quickly recover if the heading is off.
    m_driverController.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    m_driverController.back().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Operator controls for game piece intake and transfer.
    c.b().whileTrue(new IntakeSpin(intake));
    c.a().whileTrue(new IndexSpin(indexer));

    // Operator controls for intake arm motion.
    c.leftBumper().whileTrue(new raiseIntake(intakeArm));
    c.rightBumper().whileTrue(new lowerIntake(intakeArm));

    // Shooter control choices:
    // - LT/X/Y = RPM-based NEO shooter presets
    // - RT     = Limelight/Talon closed-loop shooter
    // - Start  = reverse the NEO shooter to clear jams
    c.leftTrigger().whileTrue(new shooterCommand(shooterN));
    c.x().whileTrue(new closeShooter(shooterN));
    c.y().whileTrue(new closeNeo2(shooterN));
    c.rightTrigger().whileTrue(new shooterLime(shooter));
    c.start().whileTrue(new reverseShooter(shooterN));

    // POV buttons toggle timed climb macros so one press starts the sequence and a second press cancels it.
    c.povUp().toggleOnTrue(new ClimbCommand(elevator));
    c.povDown().toggleOnTrue(new DescendCommand(elevator));

    // Feed drivetrain state into dashboard widgets and CTRE SignalLogger every loop.
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /** Publish static drive-related numbers once so students can see the configured scales on the dashboard. */
  private void publishStaticTelemetry() {
    SmartDashboard.putNumber("Drive/MaxSpeedMps", MaxSpeed);
    SmartDashboard.putNumber("Drive/MaxAngularRateRadPerSec", MaxAngularRate);
    SmartDashboard.putBoolean("Drive/FieldRelative", fieldRelative);
    SmartDashboard.putString("Drive/SpeedMode", speedModeLabel);
  }

  /**
   * Build the autonomous command selected on the SmartDashboard chooser.
   *
   * <p>The chooser stores string keys because they are easy to publish to the dashboard. This method
   * is the translation layer from those keys to actual command objects.
   */
  public Command getAutonomousCommand() {
    String Choice = autoChooser.getSelected();
    Command auto;
    switch (Choice) {
      case "leftAuto":
        auto = new leftAuto(drivetrain, shooterN, shooter, intakeArm, indexer, intake, limeShooter);
        break;
      case "leftDriveAuto":
        auto = new leftDriveAuto(drivetrain, shooterN, shooter, limeShooter, indexer, intakeArm);
        break;
      case "goToMidAuto":
        auto = new goToMidAuto(drivetrain, shooterN, shooter, intakeArm, indexer, intake, limeShooter);
        break;
      case "fromRightBumpAuto":
        auto = new fromRightBumpAuto(drivetrain, shooterN, shooter, limeShooter, indexer, intakeArm);
        break;
      case "StartAtHubAuto":
        auto = new StartAtHubAuto(drivetrain, shooterN, shooter, limeShooter, indexer);
        break;
      case "justShootAuto":
        auto = new justShootAuto(drivetrain, shooterN, shooter, limeShooter, indexer, intakeArm);
        break;
      default:
        // If nothing is selected, fall back to the raw PathPlanner auto named "leftAuto".
        auto = new PathPlannerAuto("leftAuto");
        break;
    }
    return auto;
  }

  /**
   * Helper for building either field-centric or robot-centric drive requests.
   *
   * <p>This method reflects the intended design for the drive controls: all joystick scaling and
   * the field/robot-centric choice live in one place. Right now the active default command above
   * bypasses this helper, but it is still useful to read because it shows how the control state is
   * meant to be applied.
   */
  private SwerveRequest buildDriveRequest(double xInput, double yInput, double rotInput) {
    double vx = xInput * MaxSpeed * speedModeScalar;
    double vy = yInput * MaxSpeed * speedModeScalar;
    double omega = rotInput * MaxAngularRate * speedModeScalar;

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

  /** Remember the selected speed mode and mirror the choice to SmartDashboard. */
  private void setSpeedMode(double scalar, String label) {
    speedModeScalar = scalar * OperatorConstants.drivetrainSpeedCap;
    speedModeLabel = label;
    SmartDashboard.putString("Drive/SpeedMode", speedModeLabel);
  }
}
