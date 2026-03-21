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
 * Subsystem wrapper for the climber Limelight.
 *
 * <p>This mirrors the structure of {@link LimelightShooter}, but keeps a separate table name and its
 * own cached values so a second camera can be used without mixing data sources.
 */
public class LimelightClimber extends SubsystemBase {
  public static final String LL_NAME = "limelight-climber";

  private final NetworkTable table;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private final NetworkTableEntry tv;

  private double x;
  private double y;
  private double area;
  private double distX;
  private double distY;
  private double distZ;
  private double angleTargetRadians;
  private double v;
  private double robotYaw;
  private int fiducialID;
  private Pose3d targetPose;
  private Pose3d botPose;
  private Rotation3d targetRotation;
  private Rotation3d botRotation;

  public LimelightClimber() {
    table = NetworkTableInstance.getDefault().getTable(LL_NAME);
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tv = table.getEntry("tv");
  }

  /** Read the latest Limelight snapshot and clear derived values if no valid target is visible. */
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

  /** Publish climber-camera telemetry under the {@code Limelight/Climber/...} dashboard keys. */
  public void updateDashboard() {
    SmartDashboard.putBoolean("Limelight/Climber/Valid", v > 0.5);
    SmartDashboard.putNumber("Limelight/Climber/XDegrees", x);
    SmartDashboard.putNumber("Limelight/Climber/YDegrees", y);
    SmartDashboard.putNumber("Limelight/Climber/Area", area);
    SmartDashboard.putNumber("Limelight/Climber/DistanceX", distX);
    SmartDashboard.putNumber("Limelight/Climber/DistanceY", distY);
    SmartDashboard.putNumber("Limelight/Climber/DistanceZ", distZ);
    SmartDashboard.putNumber("Limelight/Climber/TargetYawRadians", angleTargetRadians);
    SmartDashboard.putNumber("Limelight/Climber/FiducialId", fiducialID);
    SmartDashboard.putNumber("Limelight/Climber/RobotYawRadians", robotYaw);
    SmartDashboard.putString("Limelight/Climber/Status", v > 0.5 ? "Target Acquired" : "No Target");
  }

  @Override
  public void periodic() {
    updateValues();
    updateDashboard();
  }

  /** Accessors for commands that need climber vision data. */
  public boolean hasTarget() {
    return v > 0.5;
  }

  public double getDistZ() {
    return distZ;
  }

  public double getAngleTargetRadians() {
    return angleTargetRadians;
  }

  public double getAngleTargetDegrees() {
    return Units.radiansToDegrees(angleTargetRadians);
  }

  public double getRobotYaw() {
    return robotYaw;
  }

  public double getTV() {
    return v;
  }

  public double getTxDegrees() {
    return x;
  }
}
