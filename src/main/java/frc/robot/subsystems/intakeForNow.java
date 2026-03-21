package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake arm / pivot motor subsystem.
 *
 * <p>The class name is temporary, but the mechanism role is clear from how it is used:
 * {@link frc.robot.commands.raiseIntake} and {@link frc.robot.commands.lowerIntake} command this
 * motor to move the intake assembly up and down.
 *
 * <p>Right now the subsystem is open-loop:
 * <ul>
 *   <li>{@link #spin()} drives one direction</li>
 *   <li>{@link #spinTheOtherWay()} drives the opposite direction</li>
 *   <li>{@link #dontSpin()} stops the motor</li>
 * </ul>
 *
 * <p>A {@link SparkMaxConfig} and closed-loop controller object are created as a starting point for
 * future position control, but the config is not currently applied and no encoder setpoint logic is
 * active yet.
 */
public class intakeForNow extends SubsystemBase {
  private final SparkMax intake;
  private final SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;

  public intakeForNow() {
    intake = new SparkMax(20, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();

    // Prepare the config for future closed-loop work, even though this robot currently drives the arm open-loop.
    motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(0);

    // If/when closed-loop arm control is added, this line can be re-enabled to push the config to the controller.
    // intake.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Drive the intake arm in the "lower" direction used by {@code lowerIntake}. */
  public void spin() {
    intake.set(1);
  }

  /** Drive the intake arm in the opposite direction used by {@code raiseIntake}. */
  public void spinTheOtherWay() {
    intake.set(-1.0);
  }

  /** Stop the intake arm motor. */
  public void dontSpin() {
    intake.set(0);
  }

  @Override
  public void periodic() {}
}
