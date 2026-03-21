package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Single-motor indexer that advances a game piece from the intake toward the shooter.
 *
 * <p>This subsystem is intentionally simple: it exposes one "run" direction and one stop command.
 * Higher-level commands decide when the indexer should feed a note. That makes the scheduler logic
 * easy to read because button bindings and autos explicitly show when feeding starts and stops.
 */
public class Indexer extends SubsystemBase {
  private final SparkMax indexerMotor;

  public Indexer() {
    indexerMotor = new SparkMax(17, MotorType.kBrushless);
  }

  /** Feed the note toward the shooter. The sign of this output depends on motor wiring/gearbox direction. */
  public void spin() {
    indexerMotor.set(-0.4);
  }

  /** Stop the indexer so the note stays where it is. */
  public void dontspin() {
    indexerMotor.set(0);
  }

  @Override
  public void periodic() {
    // No sensor feedback is used yet, so there is nothing periodic to update.
  }
}
