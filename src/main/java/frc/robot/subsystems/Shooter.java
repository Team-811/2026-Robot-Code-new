package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Limelight-assisted TalonFX shooter.
 *
 * <p>This is the more "automatic" of the two shooter implementations in the project. Its job is to:
 * <ol>
 *   <li>read target information from the {@code limelight-shooter} NetworkTables entries</li>
 *   <li>estimate shot distance from the camera mounting geometry and Limelight {@code ty}</li>
 *   <li>look up an RPM from an interpolation table</li>
 *   <li>command the primary TalonFX in velocity closed-loop mode</li>
 *   <li>run the second TalonFX as an opposed-direction follower</li>
 * </ol>
 *
 * <p>This gives the team a way to ask for "shoot based on what the camera sees" instead of relying
 * only on fixed open-loop presets.
 */
public class Shooter extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(55, "*");
  private final TalonFX shooterMotorTheSecond = new TalonFX(43, "*");
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private static final String LIMELIGHT_NAME = "limelight-shooter";

  // Limelight mounting geometry used for the simple trig-based distance estimate.
  private static final double LIMELIGHT_HEIGHT = 0.80;
  private static final double TARGET_HEIGHT = 2.10;
  private static final double LIMELIGHT_ANGLE = Units.degreesToRadians(25);

  private static final double VELOCITY_TOLERANCE_RPS = 1.5;
  private static final double speed = 1800;

  private final NetworkTable limelightTable =
      NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);

  // Distance (meters) -> shooter wheel RPM calibration points.
  // InterpolatingDoubleTreeMap lets us ask for intermediate distances without manually adding every value.
  private final InterpolatingDoubleTreeMap distanceToRPM =
      new InterpolatingDoubleTreeMap();

  private double targetRPM = 0;

  public Shooter() {
    // Slot 0 is Phoenix's primary closed-loop gain slot for velocity control.
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 0.12;
    slot0.kI = 0;
    slot0.kD = 0;
    slot0.kV = 0.12;

    shooterMotor.getConfigurator().apply(slot0);

    // The second shooter motor mirrors the primary in the opposite direction so the wheels spin against each other.
    shooterMotorTheSecond.setControl(
        new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    shooterMotorTheSecond.getConfigurator().apply(slot0);

    // Shooter calibration points recorded on 3/19/2026.
    //distanceToRPM.put(1.8, -1275.0);
    //distanceToRPM.put(2.6, -3700.0);
    //distanceToRPM.put(3.4, -4750.0);
    distanceToRPM.put(0.8, -1200.0);
    distanceToRPM.put(1.8, -2000.0);
    distanceToRPM.put(2.6, -3000.0);
    distanceToRPM.put(3.4, -4000.0);
    distanceToRPM.put(5.0,-4900.0);
  }

  /**
   * Estimate distance from the Limelight and command the shooter to the corresponding wheel speed.
   *
   * <p>This method is designed to be called repeatedly while a command is held, not just once.
   * Re-running it lets the subsystem continuously update the requested RPM as the robot moves.
   */
  public void runShooterWithLimelight() {
    double distance = getDistanceMeters();
    if (distance < 0) {
      stopShooter();
      return;
    }

    Double rpm = distanceToRPM.get(distance);
    if (rpm == null) {
      return;
    }

    targetRPM = rpm;

    shooterMotor.setControl(
        velocityRequest.withVelocity(targetRPM / 60.0)
    );
  }

  /**
   * Estimate distance to the target using Limelight vertical angle.
   *
   * <p>The math is the standard mounting-height trigonometry used in many FRC shooters:
   * {@code distance = heightDifference / tan(cameraAngle + ty)}.
   *
   * <p>Important detail: when {@code tv == 0}, this method does not return an invalid distance.
   * Instead it sets {@code targetRPM} to the fallback {@code speed} value and still evaluates the
   * distance formula using the current {@code ty} entry. That is the current behavior of the code as written.
   */
  private double getDistanceMeters() {
    boolean hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;
    if (!hasTarget) {
      targetRPM = speed;
    }

    double ty = limelightTable.getEntry("ty").getDouble(0);

    return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
        Math.tan(LIMELIGHT_ANGLE + Units.degreesToRadians(ty));
  }

  /** True when the measured wheel speed is close enough to the requested speed to count as "ready." */
  public boolean isAtSpeed() {
    double currentRPS = shooterMotor.getVelocity().getValueAsDouble();
    double targetRPS = targetRPM / 60.0;

    return Math.abs(currentRPS - targetRPS) < VELOCITY_TOLERANCE_RPS;
  }

  /** Stop both shooter motors immediately. */
  public void stopShooter() {
    shooterMotor.stopMotor();
    shooterMotorTheSecond.stopMotor();
  }

  /** Return the last RPM value this subsystem asked the shooter to hold. */
  public double getTargetRPM() {
    return targetRPM;
  }

  @Override
  public void periodic() {
    // Placeholder for future shooter telemetry. The code currently reads tv but does not publish it yet.
    double tv = NetworkTableInstance.getDefault()
        .getTable("limelight-shooter")
        .getEntry("tv")
        .getDouble(0);

    // System.out.println("Limelight tv: " + tv);
  }
}
