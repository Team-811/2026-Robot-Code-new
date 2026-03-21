package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * Subsystem wrapper for the scoring Limelight.
 *
 * <p>This class does two jobs:
 * <ol>
 *   <li>read raw Limelight values such as {@code tx}, {@code ty}, {@code ta}, and {@code tv}</li>
 *   <li>derive more meaningful pose-based values such as target position and robot yaw</li>
 * </ol>
 *
 * <p>The values are stored in static fields and exposed through static getters because several
 * commands in this project access them without carrying around a subsystem reference. That pattern is
 * simple, but it also means this class acts a little like a global cache. The data stays current
 * because {@link RobotContainer} constructs the subsystem and the scheduler calls {@link #periodic()}
 * every loop.
 *
 * <p>Coordinate reminder for Limelight robot-space poses:
 * <ul>
 *   <li>X = left/right</li>
 *   <li>Y = up/down</li>
 *   <li>Z = forward/back</li>
 * </ul>
 */
public class LimelightShooter extends SubsystemBase {
  public static final String LL_NAME = "limelight-shooter";

  private final NetworkTable table2;

  private static double x;
  private static double y;
  private static double area;
  private static double distX;
  private static double distY;
  private static double distZ;
  private static double angleTargetRadians;
  private static double v;
  private static double robotYaw;

  private int fiducialID;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private final NetworkTableEntry tv;
  private Pose3d targetPose;
  private Pose3d botPose;
  private Rotation3d targetRotation;
  private Rotation3d botRotation;

  public LimelightShooter() {
    table2 = NetworkTableInstance.getDefault().getTable(LL_NAME);
    tx = table2.getEntry("tx");
    ty = table2.getEntry("ty");
    ta = table2.getEntry("ta");
    tv = table2.getEntry("tv");
  }

  /**
   * Pull the latest raw and derived Limelight values into local fields.
   *
   * <p>If {@code tv} reports no valid target, the derived values are reset to zero/invalid defaults
   * so commands do not accidentally act on stale data from an older frame.
   */
  public void updateValues() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    v = tv.getDouble(0.0);

    if (v < 0.5) {
      x = y = area = 0.0;
      distX = distY = distZ = 0.0;
      angleTargetRadians = 0.0;
      robotYaw = 0.0;
      fiducialID = -1;
      return;
    }

    targetPose = LimelightHelpers.getTargetPose3d_RobotSpace(LL_NAME);
    distX = targetPose.getX();
    distY = targetPose.getY();
    distZ = targetPose.getZ();

    botPose = LimelightHelpers.getBotPose3d_wpiBlue(LL_NAME);
    botRotation = botPose.getRotation();
    robotYaw = botRotation.getZ();

    targetRotation = targetPose.getRotation();
    angleTargetRadians = targetRotation.getZ();

    fiducialID = (int) LimelightHelpers.getFiducialID(LL_NAME);
  }

  /** Publish the current cached snapshot to SmartDashboard for debugging and driver feedback. */
  public void updateDashboard() {
    SmartDashboard.putBoolean("Limelight/Shooter/Valid", v > 0.5);
    SmartDashboard.putNumber("Limelight/Shooter/XDegrees", x);
    SmartDashboard.putNumber("Limelight/Shooter/YDegrees", y);
    SmartDashboard.putNumber("Limelight/Shooter/Area", area);
    SmartDashboard.putNumber("Limelight/Shooter/DistanceX", distX);
    SmartDashboard.putNumber("Limelight/Shooter/DistanceY", distY);
    SmartDashboard.putNumber("Limelight/Shooter/DistanceZ", distZ);
    SmartDashboard.putNumber("Limelight/Shooter/TargetYawRadians", angleTargetRadians);
    SmartDashboard.putNumber("Limelight/Shooter/FiducialId", fiducialID);
    SmartDashboard.putNumber("Limelight/Shooter/RobotYawRadians", robotYaw);
    SmartDashboard.putString("Limelight/Shooter/Status", v > 0.5 ? "Target Acquired" : "No Target");
  }

  @Override
  public void periodic() {
    updateValues();
    updateDashboard();
  }

  /** True when the Limelight reports a valid target ({@code tv == 1}). */
  public static boolean hasTarget() {
    return v > 0.5;
  }

  /** Distance to the target along the robot's forward axis, in meters. */
  public static double getDistZ() {
    return distZ;
  }

  /** Target yaw in radians from the 3D robot-space pose estimate. */
  public static double getAngleTargetRadians() {
    return angleTargetRadians;
  }

  /** Target yaw in degrees from the 3D robot-space pose estimate. */
  public static double getAngleTargetDegrees() {
    return Units.radiansToDegrees(angleTargetRadians);
  }

  /** Robot yaw from the Limelight's robot pose estimate, in radians. */
  public static double getRobotYaw() {
    return robotYaw;
  }

  /** Raw {@code tv} value from Limelight ({@code 0} = no target, {@code 1} = valid target). */
  public static double getTV() {
    return v;
  }

  /** Raw Limelight {@code tx} horizontal offset in degrees. */
  public static double getTxDegrees() {
    return x;
  }
}
