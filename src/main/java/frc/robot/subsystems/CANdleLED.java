package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANdleConstants;

/**
 * Simple status indicator for the CANdle.
 * Turns solid green when the supplied target condition is true, otherwise turns off.
 */
public class CANdleLED extends SubsystemBase {
  private final CANdle candle;
  private final SolidColor greenRequest;
  private final SolidColor offRequest;
  private final BooleanSupplier hasTargetSupplier;
  private boolean lastTargetState = false;

  public CANdleLED(BooleanSupplier hasTargetSupplier) {
    this.hasTargetSupplier = hasTargetSupplier;

    candle = new CANdle(CANdleConstants.candleCanId);
    candle.getConfigurator().apply(new CANdleConfiguration());

    int endIndex = Math.max(0, CANdleConstants.ledCount - 1);
    greenRequest = new SolidColor(0, endIndex).withColor(new RGBWColor(0, 255, 0));
    offRequest = new SolidColor(0, endIndex).withColor(new RGBWColor(0, 0, 0));

    candle.setControl(offRequest);
  }

  @Override
  public void periodic() {
    boolean hasTarget = hasTargetSupplier.getAsBoolean();
    if (hasTarget != lastTargetState) {
      lastTargetState = hasTarget;
      candle.setControl(hasTarget ? greenRequest : offRequest);
    }
  }
}
