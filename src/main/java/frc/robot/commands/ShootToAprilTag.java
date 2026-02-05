package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightShooter;
import frc.robot.subsystems.Shooter;

/**
 * Command to spin the shooter motors based on distance to an AprilTag target.
 * Uses Limelight-reported distance (meters) -> feet -> table lookup -> motor percents.
 * Why: lets the driver hold a trigger and get distance-compensated shooter speed without manual RPM math.
 */
public class ShootToAprilTag extends Command {
  private final Shooter shooter;

  // Distance (ft) -> top percent, bottom percent. Tune to your shooter/ballistics.
  // Entries are sorted and will be linearly interpolated for smooth RPM changes.
  private static final double[][] DISTANCE_SPEED_TABLE = {
      {1.0,  0.30, 0.32},
      {5.0,  0.35, 0.38},
      {10.0, 0.45, 0.48},
      {15.0, 0.55, 0.58},
      {20.0, 0.65, 0.68},
      {25.0, 0.75, 0.78},
      {30.0, 0.85, 0.88}
  };

  public ShootToAprilTag(Shooter shooter, LimelightShooter limelight) {
    this.shooter = shooter;
    addRequirements(shooter, limelight);
  }

  @Override
  public void initialize() {
    // Why: surface a clear status so the driver knows the command engaged.
    SmartDashboard.putString("Shooter/Status", "Init");
  }

  @Override
  public void execute() {
    boolean hasTarget = LimelightShooter.hasTarget();
    SmartDashboard.putBoolean("Shooter/HasTarget", hasTarget);

    if (!hasTarget) {
      // Why: avoid shooting on stale/zero data; forces a valid tag before running wheels.
      shooter.stop();
      SmartDashboard.putString("Shooter/Status", "No target");
      return;
    }

    double distMeters = LimelightShooter.getDistZ();
    double distFeet = Units.metersToFeet(Math.abs(distMeters));

    // Why: guard against bogus readings so we don't command garbage outputs.
    if (distFeet <= 0.0 || Double.isNaN(distFeet)) {
      shooter.stop();
      SmartDashboard.putString("Shooter/Status", "Bad distance");
      SmartDashboard.putNumber("Shooter/DistanceFt", distFeet);
      return;
    }

    double[] speeds = lookupShooterMotorSpeeds(distFeet);

    shooter.setPercents(speeds[0], speeds[1]);

    SmartDashboard.putString("Shooter/Status", "Running");
    SmartDashboard.putNumber("Shooter/DistanceFt", distFeet);
    SmartDashboard.putNumber("Shooter/TopCmd", speeds[0]);
    SmartDashboard.putNumber("Shooter/BottomCmd", speeds[1]);
  }

  @Override
  public void end(boolean interrupted) {
    // Why: ensure shooter is safe/stopped when driver releases or command is interrupted.
    shooter.stop();
    SmartDashboard.putString("Shooter/Status", interrupted ? "Interrupted" : "Finished");
  }

  @Override
  public boolean isFinished() {
    // Why: run continuously while held; driver controls lifetime via button/trigger.
    return false;
  }
  
  /**
   * Piecewise linear interpolation over the speed table. Clamps outside the table range.
   * Why: smooths RPM changes between tuned distances so shots stay consistent without big jumps.
   */
  private double[] lookupShooterMotorSpeeds(double distanceFeet) {
    // Below minimum: clamp to first row.
    if (distanceFeet <= DISTANCE_SPEED_TABLE[0][0]) {
      return new double[] {DISTANCE_SPEED_TABLE[0][1], DISTANCE_SPEED_TABLE[0][2]};
    }

    // Between rows: blend linearly for smooth RPM changes.
    for (int i = 0; i < DISTANCE_SPEED_TABLE.length - 1; i++) {
      double[] low = DISTANCE_SPEED_TABLE[i];
      double[] high = DISTANCE_SPEED_TABLE[i + 1];
      if (distanceFeet <= high[0]) {
        double t = (distanceFeet - low[0]) / (high[0] - low[0]);
        double top    = lerp(low[1], high[1], t);
        double bottom = lerp(low[2], high[2], t);
        return new double[] {top, bottom};
      }
    }

    // Above maximum: clamp to last row.
    double[] last = DISTANCE_SPEED_TABLE[DISTANCE_SPEED_TABLE.length - 1];
    return new double[] {last[1], last[2]};
  }

  private double lerp(double a, double b, double t) {
    return a + (b - a) * t;
  }
}
