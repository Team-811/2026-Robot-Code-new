package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.LimelightHelpers;

/**
 * Dedicated Limelight4 for the climber station/alignments.
 * Mirrors LimelightShooter but uses its own NT table name and isolated state to avoid collisions.
 */
public class LimelightClimber extends SubsystemBase {
    // Unique name for the climber camera; set this to match the Limelight's configured name.
    public static final String LL_NAME = "limelight-climber";

    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final NetworkTableEntry tv;

    private double x, y, area, distX, distY, distZ, angleTargetRadians, v, robotYaw;
    private int fiducialID;
    private Pose3d targetPose, botPose;
    private Rotation3d targetRotation, botRotation;

    public LimelightClimber() {
        table = NetworkTableInstance.getDefault().getTable(LL_NAME);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
    }

    /**
     * Pull latest NT values and derived poses. Guards on tv to prevent stale use.
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

    /**
     * Publish current snapshot to SmartDashboard (namespaced under LimelightClimber/).
     */
    public void updateDashboard() {
        SmartDashboard.putBoolean("LimelightClimber/Valid", v > 0.5);
        SmartDashboard.putNumber("LimelightClimber/XDegrees", x);
        SmartDashboard.putNumber("LimelightClimber/YDegrees", y);
        SmartDashboard.putNumber("LimelightClimber/Area", area);
        SmartDashboard.putNumber("LimelightClimber/DistanceX", distX);
        SmartDashboard.putNumber("LimelightClimber/DistanceY", distY);
        SmartDashboard.putNumber("LimelightClimber/DistanceZ", distZ);
        SmartDashboard.putNumber("LimelightClimber/TargetYawRadians", angleTargetRadians);
        SmartDashboard.putNumber("LimelightClimber/FiducialId", fiducialID);
        SmartDashboard.putNumber("LimelightClimber/RobotYawRadians", robotYaw);
        SmartDashboard.putString("LimelightClimber/Status", v > 0.5 ? "Target Acquired" : "No Target");
    }

    @Override
    public void periodic() {
        updateValues();
        updateDashboard();
    }

    // Accessors for commands that need climber vision data.
    public boolean hasTarget() { return v > 0.5; }
    public double getDistZ() { return distZ; }
    public double getAngleTargetRadians() { return angleTargetRadians; }
    public double getAngleTargetDegrees() { return Units.radiansToDegrees(angleTargetRadians); }
    public double getRobotYaw() { return robotYaw; }
    public double getTV() { return v; }
    public double getTxDegrees() { return x; }
}
