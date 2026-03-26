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
 * Driver-assist command that rotates the robot to face an AprilTag.
 *
 * <p>This command is meant to be bound with {@code whileTrue(...)} on a driver button. While held,
 * it temporarily takes control of the drivetrain, commands zero translation, and uses a PID loop on
 * Limelight {@code tx} to rotate toward the center of the tag. Releasing the button hands full
 * control back to the driver's normal drive command.
 *
 * <p>Only the tags listed in {@link #TARGET_TAG_IDS} are considered while the command is active.
 * That filter is pushed into the Limelight at initialize time and cleared in {@link #end(boolean)}.
 */
public class FaceAprilTag extends Command {

  private final CommandSwerveDrivetrain myDrivetrain;
  private final PIDController yawPID;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();


  private static final int[] TARGET_TAG_IDS          = {2,4,5,10}; // Dedicated tag filter for this command: only consider these IDs when aiming.
  private static final int TAG_PIPELINE_INDEX        = 0;       // Pipeline index that runs AprilTag detection on the Limelight; adjust to your config.
  private static final double MAX_ROTATE_DEG_PER_SEC = 90.0;    // Faster max rate but with stronger damping and tighter tolerance to reduce overshoot.
 
  
  private int onTargetCycles = 0;
  // Slew limiter to smooth rotational rate commands near the target.
  private final edu.wpi.first.math.filter.SlewRateLimiter omegaSlew =
      new edu.wpi.first.math.filter.SlewRateLimiter(Math.PI); // rad/s^2


  public FaceAprilTag(CommandSwerveDrivetrain drivetrain, LimelightShooter limelight) {
    myDrivetrain = drivetrain;
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
    if (LimelightShooter.hasTarget() == false) {
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
    double maxRotateDegPerSec = MAX_ROTATE_DEG_PER_SEC * myDrivetrain.getDriveModeRotationScale();
    double omegaRad =
        Units.degreesToRadians(
            MathUtil.clamp(omegaDeg, -maxRotateDegPerSec, maxRotateDegPerSec));
    omegaRad = omegaSlew.calculate(omegaRad);

    double pidErrorDeg = yawPID.getError();
    boolean atSetpoint = yawPID.atSetpoint();
    double tv = LimelightShooter.getTV();
    double pipeline = LimelightHelpers.getCurrentPipelineIndex(LimelightShooter.LL_NAME);

    myDrivetrain.setControl(
        driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(omegaRad));

    // Live debug telemetry for aiming.
    SmartDashboard.putBoolean("FaceAprilTag/Active", true);
    SmartDashboard.putNumber("FaceAprilTag/TxDegrees", txDeg);
    SmartDashboard.putNumber("FaceAprilTag/OmegaCmdRad", omegaRad);
    SmartDashboard.putNumber("FaceAprilTag/PidErrorDeg", pidErrorDeg);
    SmartDashboard.putBoolean("FaceAprilTag/AtSetpoint", atSetpoint);
    SmartDashboard.putNumber("FaceAprilTag/TvRaw", tv);
    SmartDashboard.putNumber("FaceAprilTag/Pipeline", pipeline);

    // Track on-target dwell to prevent a single good frame from ending the command.
    if (atSetpoint) {
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
    // This command is bound with whileTrue on the driver B button. Let button release end it
    // to avoid cancel/restart loops if we momentarily hit the setpoint while held.
    return false;
  }
}
