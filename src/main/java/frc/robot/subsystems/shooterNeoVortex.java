package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * NEO Vortex shooter driven by two Spark Flex controllers in velocity mode.
 *
 * <p>This subsystem now uses each motor's built-in encoder and the SPARK closed-loop controller so
 * the preset shots are expressed in RPM rather than open-loop percent output. That makes the Vortex
 * shooter behave more like the TalonFX-based {@link Shooter}: both subsystems now ask the motor
 * controllers to hold a target speed instead of just applying a raw percentage.
 *
 * <p>The preset method names are kept unchanged so the existing commands and controller bindings do
 * not need to change.
 */
public class shooterNeoVortex extends SubsystemBase {
  private static final int TOP_ID = 28;
  private static final int BOTTOM_ID = 29;

  // Approximate free speed for a NEO Vortex. These RPM presets are derived from the old percent outputs
  // so the robot keeps roughly the same tuning starting point after switching to velocity mode.
  private static final double NEO_VORTEX_FREE_SPEED_RPM = 6784.0;

  private static final double MAIN_TOP_RPM     = 0.00  * NEO_VORTEX_FREE_SPEED_RPM;
  private static final double MAIN_BOTTOM_RPM  = 0.20  * NEO_VORTEX_FREE_SPEED_RPM;
  private static final double CLOSE_TOP_RPM    = 0.00  * NEO_VORTEX_FREE_SPEED_RPM;
  private static final double CLOSE_BOTTOM_RPM = 0.10  * NEO_VORTEX_FREE_SPEED_RPM;
  private static final double CLEAR_RPM        = -0.30 * NEO_VORTEX_FREE_SPEED_RPM;

  // Starter gains for closed-loop velocity control.
  // kV is the approximate volts-per-RPM needed to reach free speed, and kP trims residual error.
  private static final double VELOCITY_kP = 0.0003;
  private static final double VELOCITY_kV = 12.0 / NEO_VORTEX_FREE_SPEED_RPM;

  private final SparkFlex topNeo;
  private final SparkFlex bottomNeo;
  private final SparkClosedLoopController topController;
  private final SparkClosedLoopController bottomController;
  private final RelativeEncoder topEncoder;
  private final RelativeEncoder bottomEncoder;

  private double targetTopRpm = 0.0;
  private double targetBottomRpm = 0.0;

  public shooterNeoVortex() {
    topNeo = new SparkFlex(TOP_ID, MotorType.kBrushless);
    bottomNeo = new SparkFlex(BOTTOM_ID, MotorType.kBrushless);

    SparkFlexConfig topConfig = createVelocityConfig();
    SparkFlexConfig bottomConfig = createVelocityConfig();

    topNeo.configure(topConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomNeo.configure(bottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    topController = topNeo.getClosedLoopController();
    bottomController = bottomNeo.getClosedLoopController();
    topEncoder = topNeo.getEncoder();
    bottomEncoder = bottomNeo.getEncoder();

    ssssssswirly_whirly_stop();
  }

  /**
   * Main preset shot used by {@link frc.robot.commands.shooterCommand}.
   *
   * <p>The top and bottom wheels intentionally use different RPMs to preserve the previous "faster
   * top wheel" behavior.
   */
  public void ssssssswirly_whirly() {
    setVelocityTargets(MAIN_TOP_RPM, MAIN_BOTTOM_RPM);
  }

  /** Lower-power preset used by {@link frc.robot.commands.closeShooter}. */
  public void close() {
    setVelocityTargets(CLOSE_TOP_RPM, CLOSE_BOTTOM_RPM);
  }

  /** Alternate close-range preset used by {@link frc.robot.commands.closeNeo2}. */
  public void close2() {
    setVelocityTargets(CLOSE_TOP_RPM, CLOSE_BOTTOM_RPM);
  }

  /** Stop both shooter motors and clear the remembered setpoints. */
  public void ssssssswirly_whirly_stop() {
    targetTopRpm = 0.0;
    targetBottomRpm = 0.0;
    topNeo.stopMotor();
    bottomNeo.stopMotor();
  }

  /** Reverse both wheels at a fixed RPM to spit out or clear a jammed game piece. */
  public void spinTheOtherWay() {
    setVelocityTargets(CLEAR_RPM, CLEAR_RPM);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterNeo/TopTargetRPM", targetTopRpm);
    SmartDashboard.putNumber("ShooterNeo/BottomTargetRPM", targetBottomRpm);
    SmartDashboard.putNumber("ShooterNeo/TopActualRPM", topEncoder.getVelocity());
    SmartDashboard.putNumber("ShooterNeo/BottomActualRPM", bottomEncoder.getVelocity());
  }

  /** Common Spark Flex configuration for velocity control using the built-in motor encoder. */
  private SparkFlexConfig createVelocityConfig() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(80);
    config.encoder.velocityConversionFactor(1.0); // Keep native units as RPM for readability.
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.pid(VELOCITY_kP, 0.0, 0.0);
    config.closedLoop.feedForward.kV(VELOCITY_kV);
    return config;
  }

  /** Command both shooter motors in closed-loop velocity mode using RPM setpoints. */
  private void setVelocityTargets(double topRpm, double bottomRpm) {
    targetTopRpm = topRpm;
    targetBottomRpm = bottomRpm;
    topController.setSetpoint(topRpm, ControlType.kVelocity);
    bottomController.setSetpoint(bottomRpm, ControlType.kVelocity);
  }
}
