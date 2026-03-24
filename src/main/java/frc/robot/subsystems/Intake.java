package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake roller subsystem.
 *
 * <p>This motor is responsible for pulling the game piece into the robot. The code currently drives
 * it open-loop at nearly full output, which is common for an intake roller when no sensor-based
 * control is needed.
 */
public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor;

  public Intake() {
    intakeMotor = new SparkMax(16, MotorType.kBrushless);
  }

  /**
   * Run the intake inward at nearly full power.
   *
   * <p>The output is intentionally close to 1.0, meaning the motor is commanded to almost full
   * applied output whenever the intake command is held.
   */
  public void spin() {
    intakeMotor.set(0.80);
  }

  /** Stop the intake roller. */
  public void dontspin() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // No sensors are currently attached, so this subsystem has no periodic telemetry/update work.
  }
}
