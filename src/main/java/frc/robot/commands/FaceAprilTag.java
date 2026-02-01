package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Usage: Bind to a button (e.g., driver B). Hold to rotate in place until the Limelight tx is ~0; releasing ends it.
 * Requirements: LimelightShooter subsystem must be scheduled so its static getters are fresh; drivetrain must be a CTRE swerve.
 * Behavior:
 * - Zeroes X/Y velocity; only commands rotation.
 * - Stops immediately when no target (tv == 0) to avoid spinning on stale data.
 * - PID on tx (deg) with deadband, tolerance, clamp, and slew to keep rotation smooth.
 * - Finishes after ON_TARGET_CYCLES_REQUIRED consecutive cycles within tolerance.
 */
public class FaceAprilTag extends Command {

  private final CommandSwerveDrivetrain myDrivetrain;
  private final LimelightShooter limelight;
  private final PIDController yawPID;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();


  private static final int[] TARGET_TAG_IDS          = {9, 11}; // Dedicated tag filter for this command: only consider these IDs when aiming.
  private static final int TAG_PIPELINE_INDEX        = 0;       // Pipeline index that runs AprilTag detection on the Limelight; adjust to your config.
  private static final double MAX_ROTATE_DEG_PER_SEC = 90.0;    // Faster max rate but with stronger damping and tighter tolerance to reduce overshoot.
  private static final int ON_TARGET_CYCLES_REQUIRED = 4;       // slight reduction for snappier finish

  
  private int onTargetCycles = 0;
  // Slew limiter to smooth rotational rate commands near the target.
  private final edu.wpi.first.math.filter.SlewRateLimiter omegaSlew =
      new edu.wpi.first.math.filter.SlewRateLimiter(Math.PI); // rad/s^2


  public FaceAprilTag(CommandSwerveDrivetrain drivetrain, LimelightShooter limelight) {
    myDrivetrain = drivetrain;
    this.limelight = limelight;
    // PID: stronger P/D for quicker arrest, small I to trim bias, continuous input for wrap.
    yawPID = new PIDController(1.4, 0.05, 0.18);
    yawPID.setSetpoint(0.0); // zero tx => aimed at tag
    yawPID.enableContinuousInput(-180.0, 180.0);
    yawPID.setTolerance(1.0); // tighter tolerance to stop nearer to center
    addRequirements(drivetrain, limelight);
  }

  /** Reset controller state each time the command is scheduled. */
  @Override
  public void initialize() {
    // Lock the Limelight into the desired AprilTag pipeline, force LEDs on for reliable detection,
    // and filter to specific IDs while this command is active.
    LimelightHelpers.setPipelineIndex(LimelightShooter.LL_NAME, TAG_PIPELINE_INDEX);
    LimelightHelpers.setLEDMode_ForceOn(LimelightShooter.LL_NAME);
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightShooter.LL_NAME, TARGET_TAG_IDS);
    yawPID.reset();
    onTargetCycles = 0;
  }

  /** Run every scheduler tick: read tx, compute omega, command rotation-only, and track dwell-on-target. */
  @Override
  public void execute() {
    // If no target, hold still to avoid spinning on stale data.
    if (LimelightShooter.getTV() == 0) {
      yawPID.reset(); // clear accumulated error when target is lost
      onTargetCycles = 0;
      myDrivetrain.setControl(
          driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      return;
    }

    // Small deadband on tx to prevent micro-oscillation; PID still handles outside tolerance.
    double txDeg = MathUtil.applyDeadband(LimelightShooter.getTxDegrees(), 0.5);
    double omegaDeg = yawPID.calculate(txDeg);
    // Clamp to a gentle max and slew-limit for smooth approach; protects hardware and driver comfort.
    double omegaRad = Units.degreesToRadians(MathUtil.clamp(omegaDeg, -MAX_ROTATE_DEG_PER_SEC, MAX_ROTATE_DEG_PER_SEC));
    omegaRad = omegaSlew.calculate(omegaRad);

    myDrivetrain.setControl(
        driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(omegaRad));

    // Live debug telemetry for aiming.
    SmartDashboard.putBoolean("FaceAprilTag/Active", true);
    SmartDashboard.putNumber("FaceAprilTag/TxDegrees", txDeg);
    SmartDashboard.putNumber("FaceAprilTag/OmegaCmdRad", omegaRad);

    // Track on-target dwell to prevent a single good frame from ending the command.
    if (yawPID.atSetpoint()) {
      onTargetCycles++;
    } else {
      onTargetCycles = 0;
    }
  }

  /** On exit (normal or interrupted), ensure we stop rotating. */
  @Override
  public void end(boolean interrupted) {
    // Stop motion, mark inactive, and release tag filter so other commands see all tags again.
    myDrivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightShooter.LL_NAME, new int[] {});
    LimelightHelpers.setLEDMode_PipelineControl(LimelightShooter.LL_NAME); // hand control back to pipeline settings
    SmartDashboard.putBoolean("FaceAprilTag/Active", false);
    SmartDashboard.putNumber("FaceAprilTag/OnTargetCycles", onTargetCycles);
  }

  /** Finish after the target is held within tolerance for the configured dwell. */
  @Override
  public boolean isFinished() {
    // Do not finish if no valid target; otherwise stop when within tolerance.
    return LimelightShooter.getTV() != 0 && onTargetCycles >= ON_TARGET_CYCLES_REQUIRED;
  }
}
