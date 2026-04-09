package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

/**
 * Intake arm / pivot motor subsystem.
 *
 * <p>The class name is temporary, but the mechanism role is clear from how it is used:
 * {@link frc.robot.commands.raiseIntake} and {@link frc.robot.commands.lowerIntake} command this
 * motor to move the intake assembly up and down.
 *
 * <p>This subsystem now uses a Kraken X60 controlled by a Phoenix 6 {@link TalonFX} on CAN ID 56.
 * The public API stays the same so the rest of the codebase does not need to care whether the arm is
 * powered by a REV or CTRE controller.
 *
 * <p>Control is still intentionally simple and open-loop:
 * <ul>
 *   <li>{@link #spin()} drives one direction at a fixed duty cycle</li>
 *   <li>{@link #spinTheOtherWay()} drives the opposite direction</li>
 *   <li>{@link #dontSpin()} stops the motor</li>
 * </ul>
 *
 * <p>If the arm moves the wrong way after wiring changes, flip the signs of the two output constants
 * below or add inversion in the TalonFX configuration.
 */
public class intakeForNow extends SubsystemBase {
  private static final double LOWER_OUTPUT = 0.15;
  private static final double RAISE_OUTPUT = -0.15;

  private final TalonFX intakeArmMotor;
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0.0);

  public intakeForNow() {
    // Use the native roboRIO CAN bus for this Kraken since it is not on a CANivore.
    intakeArmMotor = new TalonFX(OperatorConstants.cArmId);

    TalonFXConfiguration config = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true));

    intakeArmMotor.getConfigurator().apply(config);
    dontSpin();
  }

  /** Drive the intake arm in the "lower" direction used by {@code lowerIntake}. */
  public void spin() {
    intakeArmMotor.setControl(dutyRequest.withOutput(LOWER_OUTPUT));
  }

  /** Drive the intake arm in the opposite direction used by {@code raiseIntake}. */
  public void spinTheOtherWay() {
    intakeArmMotor.setControl(dutyRequest.withOutput(RAISE_OUTPUT));
  }

  /** Stop the intake arm motor. */
  public void dontSpin() {
    intakeArmMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeArm/PositionRot", intakeArmMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("IntakeArm/VelocityRps", intakeArmMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("IntakeArm/StatorCurrentA", intakeArmMotor.getStatorCurrent().getValueAsDouble());
  }
}
