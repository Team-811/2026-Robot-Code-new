package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Open-loop NEO Vortex shooter driven by two Spark Flex controllers.
 *
 * <p>This subsystem is separate from {@link Shooter}. The Talon-based {@code Shooter} attempts to
 * hold a target velocity using Limelight distance data, while this class simply applies preset
 * percent outputs. That makes it useful for quick manual shots, testing motor direction, or clearing
 * jams without relying on vision.
 *
 * <p>The method names are legacy/team-local names. The comments below explain what each one means in
 * plain robot-language.
 */
public class shooterNeoVortex extends SubsystemBase {
  private static final int TOP_ID = 28;
  private static final int BOTTOM_ID = 29;

  private static final double SPEED_CLEAR = -0.3;

  private final SparkFlex topNeo;
  private final SparkFlex bottonNeo;

  public shooterNeoVortex() {
    topNeo = new SparkFlex(TOP_ID, MotorType.kBrushless);
    bottonNeo = new SparkFlex(BOTTOM_ID, MotorType.kBrushless);

    topNeo.set(0);
    bottonNeo.set(0);
  }

  /**
   * Main preset shot used by {@link frc.robot.commands.shooterCommand}.
   *
   * <p>The top and bottom wheels intentionally use slightly different outputs, which is a simple
   * way to tune ball trajectory without closed-loop velocity control.
   */
  public void ssssssswirly_whirly() {
    topNeo.set(0.3);
    bottonNeo.set(0.2);
  }

  /** Lower-power preset used by {@link frc.robot.commands.closeShooter}. */
  public void close() {
    topNeo.set(0.2);
    bottonNeo.set(0.1);
  }

  /** Alternate close-range preset used by {@link frc.robot.commands.closeNeo2}. */
  public void close2() {
    topNeo.set(0.2);
    bottonNeo.set(0.10);
  }

  /** Stop both shooter motors. */
  public void ssssssswirly_whirly_stop() {
    setBoth(0);
  }

  /** Reverse both wheels to spit out or clear a jammed game piece. */
  public void spinTheOtherWay() {
    setBoth(SPEED_CLEAR);
  }

  @Override
  public void periodic() {}

  /** Clamp and apply the same percent output to both motors. */
  private void setBoth(double percent) {
    double clamped = Math.max(-1.0, Math.min(1.0, percent));
    topNeo.set(clamped);
    bottonNeo.set(clamped);
  }
}
