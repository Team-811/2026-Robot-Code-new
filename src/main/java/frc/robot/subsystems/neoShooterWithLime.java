// package frc.robot.subsystems;

// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkClosedLoopController;

// import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class neoShooterWithLime extends SubsystemBase {

//     private final SparkFlex topMotor = new SparkFlex(28, MotorType.kBrushless);
//     private final SparkFlex bottomMotor = new SparkFlex(29, MotorType.kBrushless);

//     private final SparkClosedLoopController topController =
//         topMotor.getClosedLoopController();

//     private final SparkClosedLoopController bottomController =
//         bottomMotor.getClosedLoopController();

//     private static final String LIMELIGHT_NAME = "limelight-shooter";

//     private static final double LIMELIGHT_HEIGHT = 0.80;
//     private static final double TARGET_HEIGHT = 2.10;
//     private static final double LIMELIGHT_ANGLE = Units.degreesToRadians(25);

//     private static final double VELOCITY_TOLERANCE_RPM = 100;

//     private final NetworkTable limelightTable =
//         NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);

//     private final InterpolatingDoubleTreeMap distanceToRPM =
//         new InterpolatingDoubleTreeMap();

//     private double targetRPM = 0;

//     public neoShooterWithLime() {

//         distanceToRPM.put(2.0, 3200.0);
//         distanceToRPM.put(2.5, 3500.0);
//         distanceToRPM.put(3.0, 3800.0);
//         distanceToRPM.put(3.5, 4150.0);
//     }

//     /* ================= RUN WITH LIMELIGHT ================= */

//     public void runShooterWithLimelight() {
//         double distance = getDistanceMeters();
//         if (distance < 0) {
//             stopShooter();
//             return;
//         }

//         Double rpm = distanceToRPM.get(distance);
//         if (rpm == null) return;

//         targetRPM = rpm;

//         topController.setSetpoint(
//             targetRPM,
//             SparkFlex.ControlType.kVelocity
//         );

//         bottomController.setSetpoint(
//             targetRPM,
//             SparkFlex.ControlType.kVelocity
//         );
//     }

//     /* ================= DISTANCE ================= */

//     private double getDistanceMeters() {
//         boolean hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
//         if (!hasTarget) return -1;

//         double ty = limelightTable.getEntry("ty").getDouble(0);

//         return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
//             Math.tan(LIMELIGHT_ANGLE + Units.degreesToRadians(ty));
//     }

//     /* ================= STATUS ================= */

//     public boolean isAtSpeed() {
//         double currentRPM = topMotor.getEncoder().getVelocity();
//         return Math.abs(currentRPM - targetRPM) < VELOCITY_TOLERANCE_RPM;
//     }

//     public void stopShooter() {
//         topMotor.stopMotor();
//         bottomMotor.stopMotor();
//     }

//     @Override
//     public void periodic() {}
// }