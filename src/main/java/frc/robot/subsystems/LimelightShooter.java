package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import frc.robot.LimelightHelpers;



//y is up and down, z is forward and backward, x is left and right
/**
 * Thin wrapper around the Limelight NetworkTables/JSON helpers.
 * Periodically snapshots Limelight values into static fields for easy access by commands.
 * Notes:
+ * - Assumes Limelight name "limelight-lime"; change here if your table name differs.
 * - Data is only fresh when this subsystem is scheduled; keep it registered in the robot container.
 * - Fields are static for simplicity; if you add more cameras, refactor to instance state to avoid collisions.
 */
public class LimelightShooter extends SubsystemBase{
    // Public so commands can refer to the correct NetworkTables instance / pipeline.
    public static final String LL_NAME = "limelight-Shooter";
    NetworkTable table2;
    static double x, y, area, distX, distY, distZ, angleTargetRadians, v, robotYaw;
    int fiducialID;
    NetworkTableEntry tx, ty, ta, tv;
    Pose3d targetPose, botPose;
    Rotation3d targetRotation, botRotation;
    
    public LimelightShooter(){
        table2 = NetworkTableInstance.getDefault().getTable(LL_NAME);
        tx = table2.getEntry("tx");
        ty = table2.getEntry("ty");
        ta = table2.getEntry("ta");
        tv = table2.getEntry("tv");
    }

    /**
     * Pulls the latest NT values and derived poses. Call from periodic.
     * Guards on tv: when no target is seen, values are cleared to zeros/invalids to prevent stale use.
     */
    public void updateValues(){
        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        v = tv.getDouble(0.0);

        // If no valid target, clear derived values to avoid acting on stale data.
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

        fiducialID = (int)LimelightHelpers.getFiducialID(LL_NAME);
    }
    /**
     * Publishes current snapshot to SmartDashboard. Keeps keys namespaced under "LimelightShooter/".
     */
    public void updateDashboard(){
        //post to smart dashboard periodically
        SmartDashboard.putBoolean("LimelightShooter/Valid", v > 0.5); // tv is 0/1; publish as boolean for clarity
        SmartDashboard.putNumber("LimelightShooter/XDegrees", x);
        SmartDashboard.putNumber("LimelightShooter/YDegrees", y);
        SmartDashboard.putNumber("LimelightShooter/Area", area);
        SmartDashboard.putNumber("LimelightShooter/DistanceX", distX);
        SmartDashboard.putNumber("LimelightShooter/DistanceY", distY);
        SmartDashboard.putNumber("LimelightShooter/DistanceZ", distZ);
        SmartDashboard.putNumber("LimelightShooter/TargetYawRadians", angleTargetRadians);
        SmartDashboard.putNumber("LimelightShooter/FiducialId", fiducialID);
        SmartDashboard.putNumber("LimelightShooter/RobotYawRadians", robotYaw);
        SmartDashboard.putString("LimelightShooter/Status", v > 0.5 ? "Target Acquired" : "No Target");
    }
    @Override
    public void periodic(){
        updateValues();
        updateDashboard();
    }

    /** True when the Limelight reports a valid target (tv==1). */
    public static boolean hasTarget(){
        return v > 0.5;
    }

    /** Distance to target along robot-forward (meters). */
    public static double getDistZ(){
        return distZ;
    }

    /** Target yaw in radians relative to the robot, from pose (not tx). */
    public static double getAngleTargetRadians(){
        return angleTargetRadians;
    }

    /** Target yaw in degrees relative to the robot, from pose (not tx). */
    public static double getAngleTargetDegrees(){
        return Units.radiansToDegrees(angleTargetRadians);
    }

    /** Robot yaw from Limelight pose estimate (radians). */
    public static double getRobotYaw(){
        return robotYaw;
    }

    /** Raw tv (0/1). */
    public static double getTV(){
        return v;
    }

    /** Limelight tx in degrees (horizontal pixel offset). */
    public static double getTxDegrees(){
        return x;
    }
}
